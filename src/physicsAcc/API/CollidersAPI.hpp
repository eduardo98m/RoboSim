#pragma once

#include "physicsAcc/Colliders/Collider.hpp"
#include <hpp/fcl/shape/geometric_shapes.h> // for Convex
#include <hpp/fcl/data_types.h>
/**
 * @brief Enum representing different types of models that can be created.
 */
enum ColliderShape
{
    SPHERE,      ///< Sphere model.
    BOX,         ///< Axis-aligned bounding box.
    CONVEX_MESH, ///< Custom mesh model loaded from a file.
    CYLINDER,    ///< Cylinder model.
    CONE,
    PLANE
};

/**
 * @brief Struct holding parameters for creating a 3D model.
 */
struct ColliderParams
{
    size_t body_id;
    int collision_group = 0; // Collision group of the collider
    vec3 position = {0.0f, 0.0f, 0.0f};
    quat orientation = {0.0, 0.0, 0.0, 1.0};

    ColliderShape model_type = ColliderShape::SPHERE;

    // Shape-specific parameters
    float radius = 1.0f;
    float length = 1.0f;
    vec3 half_extents = {1.0f, 1.0f, 1.0f};
    vec3 normal = {0.0f, 1.0f, 0.0f};
    float offset = 0.0;

    std::string model_path = ""; ///< File path for custom mesh models (used for ModelType::MESH).
};

/**
 * @brief Creates a new collider in the given collider collection.
 *
 * @param cc The collider collection to add the collider to.
 * @param params The parameters of the collider to create.
 * @return The ID of the newly created collider.
 */
size_t create_collider(ColliderCollection &cc, ColliderParams params)
{
    size_t id = cc.n_colliders;
    cc.n_colliders += 1;

    // Create collision geometry based on model_type
    std::shared_ptr<hpp::fcl::CollisionGeometry> geometry;
    switch (params.model_type)
    {
    case ColliderShape::SPHERE:
    {
        geometry = std::make_shared<hpp::fcl::Sphere>(params.radius);
        break;
    }
    case ColliderShape::BOX:
    {
        geometry = std::make_shared<hpp::fcl::Box>(params.half_extents[0], params.half_extents[1], params.half_extents[2]);
        break;
    }
    case ColliderShape::CYLINDER:
    {
        geometry = std::make_shared<hpp::fcl::Cylinder>(params.radius, params.length);
        break;
    }
    case ColliderShape::CONE:
    {
        geometry = std::make_shared<hpp::fcl::Cone>(params.radius, params.length);
        break;
    }
    case ColliderShape::PLANE:
    {
        geometry = std::make_shared<hpp::fcl::Plane>(ti::to_eigen(params.normal), params.offset);
        break;
    }
    case ColliderShape::CONVEX_MESH:
    {
        std::cout << "Currently mesh are not correctly supported(?)\n";
        // Model model = LoadModel(params.model_path.c_str());
        // Mesh mesh = model.meshes[0];

        // // 1) Extract vertices
        // std::vector<hpp::fcl::Vec3f> tmp_vertices;
        // tmp_vertices.reserve(mesh.vertexCount / 3);
        // for (int i = 0; i < mesh.vertexCount; i += 3)
        // {
        //     tmp_vertices.emplace_back(
        //         mesh.vertices[i + 0],
        //         mesh.vertices[i + 1],
        //         mesh.vertices[i + 2]);
        // }

        // // 2) Allocate FCL-owned arrays
        // const unsigned int num_points = static_cast<unsigned int>(tmp_vertices.size());
        // const unsigned int num_polygons = mesh.triangleCount;

        // // raw points array — FCL will delete[] it
        // hpp::fcl::Vec3f *points = new hpp::fcl::Vec3f[num_points];
        // for (unsigned int i = 0; i < num_points; ++i)
        //     points[i] = tmp_vertices[i];

        // // raw polygon array: one hpp::fcl::Triangle per triangle
        // auto *polygons = new hpp::fcl::Triangle[num_polygons];
        // for (unsigned int f = 0; f < num_polygons; ++f)
        // {
        //     unsigned int i0 = mesh.indices[f * 3 + 0];
        //     unsigned int i1 = mesh.indices[f * 3 + 1];
        //     unsigned int i2 = mesh.indices[f * 3 + 2];
        //     polygons[f].set(i0, i1, i2);
        // }

        // // 3) Build the Convex<> and cast it into your CollisionGeometry ptr
        // // geometry = std::shared_ptr<hpp::fcl::CollisionGeometry>(
        // //     new hpp::fcl::Convex<hpp::fcl::Triangle>(
        // //         /* ownStorage   = */ true,
        // //         /* points_      = */ points,
        // //         /* num_points_  = */ num_points,
        // //         /* polygons_    = */ polygons,
        // //         /* num_polygons_= */ num_polygons
        // //     )
        // // );

        // hpp::fcl::Convex<hpp::fcl::Triangle> geom = hpp::fcl::Convex<hpp::fcl::Triangle>(
        //     /* ownStorage   = */ true,
        //     /* points_      = */ points,
        //     /* num_points_  = */ num_points,
        //     /* polygons_    = */ polygons,
        //     /* num_polygons_= */ num_polygons)

        // geometry = std::make_shared<hpp::fcl::Convex<hpp::fcl::Triangle>>(geom);
        break;
    }
    default:
    {
        std::cerr << "Error: Unknown collider shape." << std::endl;
        return -1; // Or throw an exception
    }
    }

    // Create collision object and set its properties
    // hpp::fcl::CollisionObject collision_object(geometry, ti::get_eigen_transform(params.position, params.orientation));

    std::shared_ptr<hpp::fcl::CollisionObject> collision_object = std::make_shared<hpp::fcl::CollisionObject>(
        geometry, ti::get_eigen_transform(params.position, params.orientation));

    ColliderUserData *u_data = new ColliderUserData{
        .body_id = params.body_id,
        .collision_group = params.collision_group};

    collision_object->setUserData(u_data);

    // Register the object with the manager
    cc.collision_manager.registerObject(collision_object.get());

    // Add the collision object to the collider collection
    cc.collider.push_back(collision_object);

    return id;
}