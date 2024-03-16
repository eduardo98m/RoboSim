#include "broad_phase.hpp"

AABB compute_AABB(std::shared_ptr<hpp::fcl::CollisionGeometry> shape, const vec3 &position, const quat &orientation){
    
    hpp::fcl::CollisionObject obj = hpp::fcl::CollisionObject(shape, ti::get_eigen_transform(position, orientation));
    
    obj.computeAABB();
    hpp::fcl::AABB aabb =  obj.getAABB();

    AABB new_aabb = {
        .min =  {aabb.min_[0],aabb.min_[1],aabb.min_[2]},
        .max =  {aabb.max_[0],aabb.max_[1],aabb.max_[2]}};
    
    return new_aabb;
}

AABB compute_AABB(const Sphere &sphere, const vec3 &position, const quat &orientation)
{

    AABB aabb = {
        .min = position - sphere.radius,
        .max = position + sphere.radius};
    return aabb;
}

AABB compute_AABB(const Box &box, const vec3 &position, const quat &orientation)
{
    // Rotate each corner of the box
    vec3 min = position, max = position;
    for (int i = 0; i < 8; ++i) {
        // Note about the vec3(i & 1 ? -1 : 1, i & 2 ? -1 : 1, i & 4 ? -1 : 1), expresion below
        vec3 corner = box.half_extents * vec3(i & 1 ? -1 : 1, 
                                              i & 2 ? -1 : 1, 
                                              i & 4 ? -1 : 1);
        vec3 rotated_corner = ti::rotate(orientation, corner) + position;
        min = ti::min(min, rotated_corner);
        max = ti::max(max, rotated_corner);
    }

    // The expression vec3(i & 1 ? -1 : 1, i & 2 ? -1 : 1, i & 4 ? -1 : 1) is used to generate the eight corners of the box. 
    // & is the bitwise and: i & 1, i & 2, i & 4
    // Check the 1st, 2nd, and 3rd bits of i, respectively.
    // If the bit is set (i.e., it is 1), the result of the AND operation is non-zero, and the ternary operator ? -1 : 1 returns -1. Otherwise, it returns 1.
    
    // Compute the axis-aligned bounding box
    AABB aabb = {.min = min, .max = max};
    return aabb;
}

// TODO Check this function
AABB compute_AABB(const Capsule &capsule, const vec3 &position, const quat &orientation)
{
    // Rotate the capsule axis based on the orientation
    

    // (TODO: This might be a source of errors, ensure that this is the right axis for the capsule)
    vec3 half_extents = {capsule.radius, 0.5 * capsule.height, capsule.radius};
    vec3 rotated_half_extents = ti::rotate(orientation, half_extents);
    rotated_half_extents = abs(rotated_half_extents);
    

    // Compute the axis-aligned bounding box
    AABB aabb = {
        .min = position - rotated_half_extents,
        .max = position + rotated_half_extents};
    return aabb;
}

AABB merge_aabb(const AABB& aabb1, const AABB& aabb2) {

  vec3 min_point = vec3(
    std::min(aabb1.min.x, aabb2.min.x),
    std::min(aabb1.min.y, aabb2.min.y),
    std::min(aabb1.min.z, aabb2.min.z)
  );

  vec3 max_point = vec3(
    std::max(aabb1.max.x, aabb2.max.x),
    std::max(aabb1.max.y, aabb2.max.y), 
    std::max(aabb1.max.z, aabb2.max.z)
  );

  return AABB{min_point, max_point};
}

AABB compute_AABB(const ShapeInfo &shape, const vec3 &position, const quat &orientation){
    AABB aabb = AABB{.min = vec3{0.0, 0.0, 0.0}, .max = vec3{0.0, 0.0, 0.0}};

    if (shape.type == ShapeType::CAPSULE)
    {
        aabb = compute_AABB(*shape.capsule, position, orientation);
    }

    else if (shape.type == ShapeType::SPHERE)
    {
        aabb = compute_AABB(*shape.sphere, position, orientation);
    }

    else if (shape.type == ShapeType::BOX)
    {
        aabb =  compute_AABB(*shape.box, position, orientation);
    }

    return aabb;
}



// AABB collision dectection using the separating axes theorem
bool check_broad_phase_collision(const AABB &aabb_1, const AABB &aabb_2)
{

    return (aabb_1.min.x <= aabb_2.max.x && aabb_1.max.x >= aabb_2.min.x) && // Check x axis
           (aabb_1.min.y <= aabb_2.max.y && aabb_1.max.y >= aabb_2.min.y) && // Check y axis
           (aabb_1.min.z <= aabb_2.max.z && aabb_1.max.z >= aabb_2.min.z);   // Check z axis
}

bool check_broad_phase_collision(const Plane &plane, const AABB &aabb)
{
    scalar distance_min = ti::dot(plane.normal, aabb.min) + plane.offset;
    scalar distance_max = ti::dot(plane.normal, aabb.max) + plane.offset;

    return ((distance_min >= 0 && distance_max < 0) ||
            (distance_min < 0 && distance_max >= 0));
}

bool check_broad_phase_collision(const hpp::fcl::Plane &plane, const AABB &aabb)
{
    scalar distance_min = ti::dot(ti::from_eigen(plane.n), aabb.min) + plane.d;
    scalar distance_max = ti::dot(ti::from_eigen(plane.n), aabb.max) + plane.d;

    return ((distance_min >= 0 && distance_max < 0) ||
            (distance_min < 0 && distance_max >= 0));
}

bool check_broad_phase_collision(const AABB &aabb, const Plane &plane)
{
    return check_broad_phase_collision(plane, aabb);
}