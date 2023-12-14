/**
 * @file shapes.hpp
 * @brief Defines different shapes used in physics simulations.
 */

#pragma once

#include "physics/math/math.hpp"
#include <vector>

/**
 * @enum ShapeType
 * @brief Enumerates various types of shapes that can be used in physics simulations.
 */
enum class ShapeType
{
    BOX,        ///< Box shape.
    SPHERE,     ///< Sphere shape.
    CAPSULE,    ///< Capsule shape.
    CYLINDER,   ///< Cylinder shape.
    PLANE,      ///< Plane shape.
    HEIGHTMAP   ///< Heightmap shape.
};

/**
 * @struct Box
 * @brief Defines a box shape.
 */
struct Box
{
    vec3 half_extents;  ///< Half-extents defining the size of the box.
};

/**
 * @struct Sphere
 * @brief Defines a sphere shape.
 */
struct Sphere
{
    scalar radius;  ///< Radius of the sphere.
};

/**
 * @struct Capsule
 * @brief Defines a capsule shape.
 */
struct Capsule
{
    scalar radius;  ///< Radius of the capsule.
    scalar height;  ///< Height of the capsule.
};

/**
 * @struct Cylinder
 * @brief Defines a cylinder shape.
 */
struct Cylinder
{
    scalar radius;  ///< Radius of the cylinder.
    scalar height;  ///< Height of the cylinder.
};

/**
 * @struct Plane
 * @brief Defines a plane shape.
 */
struct Plane
{
    vec3 normal;    ///< Normal vector of the plane.
    scalar offset;  ///< Offset from the origin along the normal vector.
};

/**
 * @struct Heightmap
 * @brief Defines a heightmap shape.
 */
struct Heightmap
{
    scalar width;           ///< Width of the heightmap.
    scalar depth;           ///< Depth of the heightmap.
    scalar height_scale;    ///< Scaling factor for heights.
    int width_segments;     ///< Number of width segments in the heightmap.
    int depth_segments;     ///< Number of depth segments in the heightmap.
    std::vector<scalar> heights; ///< Heights at each point on the heightmap.
};
