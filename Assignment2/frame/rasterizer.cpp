// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   // The most risky part.
    
    Eigen::Vector2f AB, BC, CA, AP, BP, CP, P;
    float crossABAP, crossBCBP, crossCACP;
    P << x, y;
    AB = _v[1].head(2) - _v[0].head(2);
    BC = _v[2].head(2) - _v[1].head(2);
    CA = _v[0].head(2) - _v[2].head(2);

    AP = P - _v[0].head(2);
    BP = P - _v[1].head(2);
    CP = P - _v[2].head(2);

    crossABAP = AB[0]*AP[1] - AP[0]*AB[1];
    crossBCBP = BC[0]*BP[1] - BP[0]*BC[1];
    crossCACP = CA[0]*CP[1] - CP[0]*CA[1];
    // if( ((crossABAP>0)==(crossBCBP>0)) && ((crossBCBP>0) == (crossCACP>0)) ) return true;
    if((crossABAP>0&&crossBCBP>0&&crossCACP>0) ||(crossABAP<0&&crossBCBP<0&&crossCACP<0))return true;
    return false;
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}
                                  
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(1.0-vert.y());
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {

    // TODO : Find out the bounding box of current triangle.
    auto v = t.toVector4();//t-v.x() t-v.y() t-v.z() t-v-1.f I think.

    float max_x = v[0].x(), min_x = v[0].x();
    if(v[1].x() > max_x) max_x = v[1].x();
    if(v[2].x() > max_x) max_x = v[2].x();

    if(v[1].x() < min_x) min_x = v[1].x();
    if(v[2].x() < min_x) min_x = v[2].x();

    float max_y= v[0].y(), min_y = v[0].y();
    if(v[1].y() > max_y) max_y = v[1].y();
    if(v[2].y() > max_y) max_y = v[2].y();

    if(v[1].y() < min_y) min_y = v[1].y();
    if(v[2].y() < min_y) min_y = v[2].y();

    //now we get the vertex of the bouding box: (min_x, min_y) and (max_x, max_y).

    // iterate through the pixel and find if the current pixel is inside the triangle

    for(int x=min_x; x<max_x; x++)
    {
        for(int y=min_y; y<max_y; y++)
        {
            if(insideTriangle(x+0.5, y+0.5, t.v))
            {
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);//get current z-value
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                //depth_buf is in rasterizer.hpp line 95.

                int cur_index = get_index(x, y);
                if(z_interpolated < depth_buf[cur_index])
                {
                    Eigen::Vector3f This_p;
                    This_p << x, y, z_interpolated;
                    set_pixel(This_p, t.getColor());
                    depth_buf[cur_index] = z_interpolated;
                }
            }
        }
    }
    // getColor is a method in Triangle.hpp line 27.
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    // old index:
    // auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on