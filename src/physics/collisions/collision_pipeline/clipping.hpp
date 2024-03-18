
#pragma once
#include "physics/math/math.hpp"
#include "collider.hpp"
#include "array"
#include "simplex.hpp"
#include "gjk.hpp"
#include "epa.hpp"


void clipping_get_contact_manifold(Collider* collider1, Collider* collider2, vec3 normal, scalar penetration, Collider_Contact** contacts);