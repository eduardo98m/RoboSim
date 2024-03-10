#pragma once
#include "physics/math/math.hpp"
#include "collider.hpp"
#include "simplex.hpp"
#include "support_function.hpp"

// Main code from : https://winter.dev/articles/gjk-algorithm

bool gjk(const Collider& collider_1, const Collider& collider_2, Simplex* simplex);
