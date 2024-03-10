#include "collider.hpp"

std::unique_ptr<Collider> cast_shape_to_collider(const ShapeInfo &shape, const vec3 &position, const quat &orientation)
{
    if (shape.type == ShapeType::SPHERE)
    {
        return std::make_unique<SphereCollider>(SphereCollider(
            position,
            shape.sphere->radius));
    }
    else if (shape.type == ShapeType::BOX)
    {
        return std::make_unique<MeshCollider>(MeshCollider(
            get_box_vertices(shape.box->half_extents, position, orientation),
            get_box_faces_indices()));
    }
    else if (shape.type == ShapeType::PLANE)
    {
        return std::make_unique<PlaneCollider>(PlaneCollider(
            shape.plane->normal, shape.plane->offset));
    }

    return nullptr;
}
