#pragma once
#include <glm/glm.hpp>


/**
 * @brief Computes the intersection point between a ray and a triangle.
 *
 * @param orig The origin of the ray.
 * @param dir The direction of the ray.
 * @param v0 The first vertex of the triangle.
 * @param v1 The second vertex of the triangle.
 * @param v2 The third vertex of the triangle.
 * @param t The intersection distance.
 *
 * @return True if the ray intersects the triangle, False otherwise.
 * 
 * Equations used:

 * Equation 1: P = orig + t * dir
 * Equation 2: d = -glm::dot(N, v0)
 * Equation 3: t = -(glm::dot(N, orig) + d) / NdotRayDirection
 *
 * where:

 * P is the intersection point
 * t is the intersection distance
 * orig is the origin of the ray
 * dir is the direction of the ray
 * N is the normal vector to the triangle
 * v0 is the first vertex of the triangle
 * d is a parameter used to compute t
 * NdotRayDirection is the dot product of N and dir

 */
bool ray_triangle_intersection(const glm::vec3& orig, const glm::vec3& dir, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, float& t);