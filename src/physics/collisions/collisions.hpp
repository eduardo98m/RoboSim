#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"
struct CollisionResponse{
    vec3 normal = {1.0, 0.0, 0.0};
    vec3 contact_point_1;
    vec3 contact_point_2;
};

// Sphere - Sphere
CollisionResponse compute_collision_response(const vec3& position_1, const vec3&  position_2, const Sphere &sphere_1, const Sphere & sphere_2);
// Box - Box
CollisionResponse compute_collision_response(const vec3 &position_1, const quat &orientation_1, const vec3 &position_2, const quat &orientation_2, const Box &box_1, const Box &box_2);
// Sphere - Box
CollisionResponse compute_collision_response(const vec3& box_position, const quat&  box_orientation, const vec3& sphere_position, const Box &box, const Sphere & sphere);

// Plane - Sphere
CollisionResponse compute_collision_response(const vec3& sphere_position, const Sphere & sphere, Plane plane);
// Box - Plane
CollisionResponse compute_collision_response(const vec3& box_position, const quat& box_orientation, const Box & box, Plane plane);


// Utils:
std::vector<vec3> get_box_vertices(const vec3 &half_extents, const vec3 &position, const quat &orientation);
std::pair<scalar, scalar> get_vertices_projection_max_and_min(const std::vector<vec3> &vertices, const vec3 &axis);
std::pair<scalar, scalar> get_projections_overlap(scalar projection_1_min, scalar projection_1_max, scalar projection_2_min, scalar projection_2_max);

