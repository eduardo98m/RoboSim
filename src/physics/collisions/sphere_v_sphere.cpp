#include "collisions.hpp"

ContactPoint compute_collision_response(const vec3& position_1, const vec3&  position_2, const Sphere &sphere_1, const Sphere & sphere_2){

    ContactPoint response = ContactPoint();

    vec3 normal = ti::normalize(position_2 - position_1);
    response.normal = normal;

    response.contact_point_1 = position_1 + normal * sphere_1.radius;

    response.contact_point_2 = position_2 - normal * sphere_2.radius;

    return response;

}