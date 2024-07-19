#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"
#include "hpp/fcl/collision_data.h"
#include "hpp/fcl/collision.h"
#include "hpp/fcl/BVH/BVH_model.h"
#include <memory>
struct AABB
{
    vec3 min = {0.0, 0.0, 0.0};
    vec3 max = {0.0, 0.0, 0.0};
};

AABB expandAABB(AABB aabb, vec3 factor);

AABB compute_AABB(std::shared_ptr<hpp::fcl::CollisionGeometry> shape, const vec3 &position, const quat &orientation);

AABB compute_AABB(const ShapeInfo &shape, const vec3 &position, const quat &orientation);

AABB compute_AABB(const Sphere &sphere, const vec3 &position, const quat &orientation);
AABB compute_AABB(const Box &box, const vec3 &position, const quat &orientation);
AABB compute_AABB(const Capsule &capsule, const vec3 &position, const quat &orientation);

AABB merge_aabb(const AABB& aabb1, const AABB& aabb2);

bool check_broad_phase_collision(const AABB &aabb_1, const AABB &aabb_2);

// AS it is imposible to compute a Planes AABB we need to use a sperate function to compare it to a AABB
bool check_broad_phase_collision(const hpp::fcl::Plane &plane, const AABB &aabb);
bool check_broad_phase_collision(const Plane &plane, const AABB &aabb);
bool check_broad_phase_collision(const AABB &aabb, const Plane &plane);