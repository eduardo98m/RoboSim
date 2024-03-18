#include ""

void clipping_get_contact_manifold(Collider* collider1, Collider* collider2, vec3 normal, scalar penetration,
	Collider_Contact** contacts) {
	// TODO: For now, we only consider CONVEX and SPHERE colliders.
	// If new colliders are added, we can think about making this more generic.

	if (collider1->type == COLLIDER_TYPE_SPHERE) {
		vec3 sphere_collision_point = support_point(collider1, normal);

		Collider_Contact contact;
		contact.collision_point1 = sphere_collision_point;
		contact.collision_point2 = gm_vec3_subtract(sphere_collision_point, gm_vec3_scalar_product(penetration, normal));
		contact.normal = normal;
		array_push(*contacts, contact);
	} else if (collider2->type == COLLIDER_TYPE_SPHERE) {
		vec3 inverse_normal = gm_vec3_invert(normal);
		vec3 sphere_collision_point = support_point(collider2, inverse_normal);

		Collider_Contact contact;
		contact.collision_point1 = gm_vec3_add(sphere_collision_point, gm_vec3_scalar_product(penetration, normal));
		contact.collision_point2 = sphere_collision_point;
		contact.normal = normal;
		array_push(*contacts, contact);
	} else {
		// For now, this case must be convex-convex
		assert(collider1->type == COLLIDER_TYPE_CONVEX_HULL);
		assert(collider2->type == COLLIDER_TYPE_CONVEX_HULL);
		convex_convex_contact_manifold(collider1, collider2, normal, contacts);
	}
}