#include "collisions.hpp"

CollisionResponseData compute_collision_response(vec3 position_1, quat orientation_1, vec3 position_2, quat orientation_2, Sphere sphere_1, Sphere sphere_2){

    CollisionResponseData response = CollisionResponseData();

    // Calculate the penetration
    scalar penetration = sphere_1.radius + sphere_2.radius - ti::magnitude(position_2 - position_1);

    if (penetration < 0){
        response.collision = false;
        return response;
    }


    response.penetration_depth = penetration;

    response.normal = ti::normalize(position_2 - position_1);

    response.collision_manifold[0] = position_1 + response.normal * sphere_1.radius;

    response.collision_points = 1;

    return response;

}