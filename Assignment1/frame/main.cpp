#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate <<
     1, 0, 0, -eye_pos[0],
     0, 1, 0, -eye_pos[1],
     0, 0, 1, -eye_pos[2],
     0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float a = rotation_angle*MY_PI/180.0; //trans rotation angle into radian.
    Eigen::Matrix4f rotation_z;
    rotation_z <<
    cos(a), -sin(a), 0, 0,
    sin(a), cos(a), 0, 0,
    0, 0, 1, 0, 
    0, 0, 0, 1;
    model = rotation_z*model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float n = -zNear;
    float f = zFar;
    float fovY = eye_fov*MY_PI / 180.0;
    float t = n*tan(fovY/2.0);
    float b = -t;
    float r = aspect_ratio*t;
    float l = -r;
    Eigen::Matrix4f Mpersp_ortho;
    Mpersp_ortho <<
    n, 0, 0, 0,
    0, n, 0, 0,
    0, 0, n+f, -n*f,
    0, 0, 1, 0;

    Eigen::Matrix4f Mortho, MorthoR, MorthoT;
    //Mortho = MorthoR*MorthoT

    MorthoT <<
    1, 0, 0, -(r+l)/2.0,
    0, 1, 0, -(t+b)/2.0,
    0, 0, 1, -(n+f)/2.0,
    0, 0, 0, 1;

    MorthoR <<
    2.0/(r-l), 0, 0, 0,
    0, 2.0/(t-b), 0, 0,
    0, 0, 2.0/(n-f), 0,
    0, 0, 0, 1;

    Mortho = MorthoR*MorthoT;
    projection = Mortho*Mpersp_ortho;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation_matrix;
    float a = (angle*MY_PI)/180.0;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Nmatrix;
    Nmatrix <<
    0, -axis[2], axis[1],
    axis[2], 0, -axis[0],
    -axis[1], axis[0], 0;
    Eigen::Matrix3f Rm;
    Rm = cos(a)*I + (1-cos(a))*axis*axis.transpose() + sin(a)*Nmatrix;
    rotation_matrix <<
    Rm(0, 0), Rm(0, 1), Rm(0, 2), 0,
    Rm(1, 0), Rm(1, 1), Rm(1, 2), 0,
    Rm(2, 0), Rm(2, 1), Rm(2, 2), 0,
    0, 0, 0, 1;
    return rotation_matrix;
}

int main(int argc, const char** argv)
{
    float x = 0;
    float y = 0;
    float z = 0;
    float angle = 0;
    float custom_angle = 0;
    bool command_line = false;
    std::cin >> x >> y >> z >> custom_angle;
    Eigen::Vector3f n_axis(x, y, z);
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(800, 800);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) 
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_rotation(n_axis, angle));
        //r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(800, 800, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
     {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(n_axis, angle));
        ///////HERE IS WHAT YOU NEED TO MODIFY!!!!!!
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        // in rasterizer.cpp, line 135, receive an triangle to draw.

        cv::Mat image(800, 800, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(2);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += custom_angle;
        }
        else if (key == 'd') {
            angle -= custom_angle;
        }
    }

    return 0;
}
