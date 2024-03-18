#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"
#include "simplex.hpp"
#include "collider.hpp"
#include "vector"
#include "support_function.hpp"
#include <algorithm>


struct EPACollisionResponse{
    vec3 normal;
	scalar penetration_depth;
	bool collision = false;
};

EPACollisionResponse epa(
    Simplex &simplex,
    const Collider &colliderA,
    const Collider &colliderB);