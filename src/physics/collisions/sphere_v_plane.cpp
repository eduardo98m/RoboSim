#include "collisions.hpp"

CollisionResponse compute_collision_response(const vec3& sphere_position, const Sphere & sphere, Plane plane){

    
    CollisionResponse response = CollisionResponse();

    //Signed distance between the nearest point of the plane to the sphere
    scalar distance = ti::dot(sphere_position, plane.normal) - plane.offset;

    response.normal = plane.normal;

    response.contact_point_1 = sphere_position - plane.normal * sphere.radius;

    response.contact_point_2 = sphere_position - plane.normal * distance;

    return response;

}