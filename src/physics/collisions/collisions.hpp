#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"

struct ContactPoint{
    vec3 normal = {1.0, 0.0, 0.0};
    vec3 contact_point_1;
    vec3 contact_point_2;
};

struct CollisionResponse{
    int contact_points = 0;
    ContactPoint contact_manifold[4];
};

// Sphere - Sphere
ContactPoint compute_collision_response(const vec3& position_1, const vec3&  position_2, const Sphere &sphere_1, const Sphere & sphere_2);
// Box - Box
ContactPoint compute_collision_response(const vec3 &position_1, const quat &orientation_1, const vec3 &position_2, const quat &orientation_2, const Box &box_1, const Box &box_2);
// Sphere - Box
ContactPoint compute_collision_response(const vec3& box_position, const quat&  box_orientation, const vec3& sphere_position, const Box &box, const Sphere & sphere);

// Plane - Sphere
ContactPoint compute_collision_response(const vec3& sphere_position, const Sphere & sphere, Plane plane);
// Box - Plane
ContactPoint compute_collision_response(const vec3& box_position, const quat& box_orientation, const Box & box, Plane plane);


// Utils:
std::vector<vec3> get_box_vertices(const vec3 &half_extents, const vec3 &position, const quat &orientation);
std::vector<std::vector<size_t>> get_box_faces_indices();
std::tuple<vec2, vec3, vec3> get_vertices_projection_max_and_min(const std::vector<vec3> &vertices, const vec3 &axis);
std::pair<scalar, scalar> get_projections_overlap(scalar projection_1_min, scalar projection_1_max, scalar projection_2_min, scalar projection_2_max);

