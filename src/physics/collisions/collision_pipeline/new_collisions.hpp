#pragma once
#include "physics/math/math.hpp"
#include "collider.hpp"
#include "array"
#include "simplex.hpp"
#include "gjk.hpp"
#include "epa.hpp"

struct Contact
{
    vec3 normal = {1.0, 0.0, 0.0};
    vec3 contact_point_1;
    vec3 contact_point_2;
}

struct ContactManifold
{
    int contact_points = 0;
    std::array<Contact, 16> manifold;
};

ContactManifold colliders_get_contacts(Collider *collider_1, Collider *colider_2);

ContactManifold clipping_get_contact_manifold(Collider* collider1, Collider* collider2, vec3 normal, scalar penetration);

ContactManifold convex_convex_contact_manifold();