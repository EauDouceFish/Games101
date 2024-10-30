//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"


struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;//initialize the fragment shader
    }

    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex) :
         color(col), normal(nor), tex_coords(tc), texture(tex) {}

    // all inputs: color, normal, texture_coordinates, texture
    Eigen::Vector3f view_pos;
    Eigen::Vector3f color;
    Eigen::Vector3f normal;
    Eigen::Vector2f tex_coords;//why color is given still need uv
    Texture* texture;
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;//only need to identify the position of the vertex
};

#endif //RASTERIZER_SHADER_H
