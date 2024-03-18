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

    for (int i = 0; i < 8; i++)
    {
        vertices[i] = ti::rotate(orientation, vertices[i]) + position;
    }

    return vertices;
}

std::vector<std::vector<size_t>> get_box_faces_indices() {
    return {
        {0, 1, 3, 2}, // Front face
        {4, 6, 7, 5}, // Back face
        {0, 4, 5, 1}, // Left face
        {2, 3, 7, 6}, // Right face
        {0, 2, 6, 4}, // Bottom face
        {1, 5, 7, 3}  // Top face
    };
}

std::tuple<vec2, vec3, vec3> get_vertices_projection_max_and_min(const std::vector<vec3> &vertices, const vec3 &axis)
{
    scalar min_projection = INFINITY;
    scalar max_projection = -INFINITY;
    vec3 min_vertex = {0.0, 0.0, 0.0};
    vec3 max_vertex = {0.0, 0.0, 0.0};

    for (int i = 0; i < 8; i++)
    {
        scalar projection = ti::dot(vertices[i], axis);

        
        if (projection < min_projection) {
            min_projection = projection;
            min_vertex = vertices[i];
        }

        if (projection > max_projection) {
            max_projection = projection;
            max_vertex = vertices[i];
        }
        
    }

    return {(vec2){min_projection, max_projection}, min_vertex, max_vertex};
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
