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

// Plane - Sphere
CollisionResponse compute_collision_response(const vec3& sphere_position, const Sphere & sphere, Plane plane);