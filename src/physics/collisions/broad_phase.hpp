#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"
struct AABB
{
    vec3 min;
    vec3 max;
};

AABB expandAABB(scalar factor);

AABB compute_AABB(const Sphere &sphere, const vec3 &position, const quat &orientation);
AABB compute_AABB(const Box &box, const vec3 &position, const quat &orientation);
AABB compute_AABB(const Capsule &capsule, const vec3 &position, const quat &orientation);

bool check_broad_phase_collision(const AABB &aabb_1, const AABB &aabb_2);

// AS it is imposible to compute a Planes AABB we need to use a sperate function to compare it to a AABB
bool check_broad_phase_collision(const Plane &plane, const AABB &aabb);
bool check_broad_phase_collision(const AABB &aabb, const Plane &plane);