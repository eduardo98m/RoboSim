#include "support_function.hpp"

vec3 support_function(const Collider& colliderA, const Collider& colliderB, vec3 direction)
{
	return colliderA.find_furthest_point( direction)
	     - colliderB.find_furthest_point(-direction);
}