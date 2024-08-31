#pragma once
#include <memory>
#include "physics/math/math.hpp"
#include "hpp/fcl/BVH/BVH_model.h"
#include "hpp/fcl/collision.h"
#include "hpp/fcl/collision_data.h"

struct Collider{
    // Collider geometry
    std::shared_ptr<hpp::fcl::CollisionGeometry> geom;
    // Position of the collider relative to the body center.
    vec3 pos = {0.0, 0.0, 0.0};
    // Orientation (rotation) of the collider relative to the body center
    quat rot = {1.0, 0.0, 0.0, 0.0};
    // Restitution
    scalar restitution = 0.5;
    // Static friction
    scalar static_fricition = 0.5;
    // Dynamic friction
    scalar dynamic_friction = 0.5;
    // Body id to which the collider is attached to.
    size_t body_id;  
};