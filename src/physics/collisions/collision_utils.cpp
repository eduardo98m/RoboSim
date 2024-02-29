#include "collisions.hpp"

std::vector<vec3> get_box_vertices(const vec3 &half_extents, const vec3 &position, const quat &orientation)
{

    scalar x = half_extents.x;
    scalar y = half_extents.y;
    scalar z = half_extents.z;

    std::vector<vec3> vertices = {
        {-x, -y, -z},
        {x, -y, -z},
        {-x, y, -z},
        {x, y, -z},
        {-x, -y, z},
        {x, -y, z},
        {-x, y, z},
        {x, y, z}};

    //std::vector<vec3> vertices(8);

    for (int i = 0; i < 8; i++)
    {
        vertices[i] = ti::rotate(orientation, vertices[i]) + position;
    }

    return vertices;
}

std::pair<scalar, scalar> get_vertices_projection_max_and_min(const std::vector<vec3> &vertices, const vec3 &axis)
{
    scalar min_projection = -INFINITY;
    scalar max_projection = INFINITY;

    for (int i = 0; i++; i < 8)
    {
        scalar projection = ti::dot(vertices[i], axis);
        min_projection = ti::min(min_projection, projection);
        max_projection = ti::max(max_projection, projection);
    }

    return {min_projection, max_projection};
}

std::pair<scalar, scalar> get_projections_overlap(
    scalar projection_1_min,
    scalar projection_1_max,
    scalar projection_2_min,
    scalar projection_2_max)
{
    scalar overlap = ti::min(projection_1_max, projection_2_max) - ti::max(projection_1_min, projection_2_min);

    // Direction of the overlap relative to the 1st collision shape entity;
    scalar direction = projection_1_min < projection_2_min ? 1.0 : -1.0;

    return {
        overlap, direction};
}
