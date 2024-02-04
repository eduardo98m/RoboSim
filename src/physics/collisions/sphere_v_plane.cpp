#include "collisions.hpp"

CollisionResponseData compute_collision_response(vec3 position_1, quat orientation_1, vec3 position_2, quat orientation_2, Sphere sphere, Plane plane){

    
    CollisionResponseData response = CollisionResponseData();

    // Distance between the nearest point of the plane to the sphere
    scalar distance = abs(ti::dot(position_1, plane.normal) - plane.offset);

    scalar penetration = sphere.radius - distance;
    
    if (penetration < 0){
        response.collision = false;
        return response;
    }

    response.penetration_depth = penetration;

    response.normal = plane.normal;

    // response.collision_manifold[0] = position_1 - response.normal * distance;

    // response.contact_point_2 = 1;

    return response;

}