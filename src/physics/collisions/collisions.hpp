#include "physics/math/math.hpp"
#include "Shapes.hpp"
struct CollisionResponseData{
    bool collision = false;
    vec3 normal = {1.0, 0.0, 0.0};
    scalar penetration_depth = 0.0;
    vec3 contact_point_1;
    vec3 contact_point_2;
};

// Sphere - Sphere
CollisionResponseData compute_collision_response(vec3 position_1, quat orientation_1, vec3 position_2, quat orientation_2, Sphere sphere_1, Sphere sphere_2);

// Plane - Sphere
CollisionResponseData compute_collision_response(vec3 position_1, quat orientation_1, vec3 position_2, quat orientation_2, Sphere sphere, Plane plane);
CollisionResponseData compute_collision_response(vec3 position_1, quat orientation_1, vec3 position_2, quat orientation_2, Plane plane, Sphere sphere);