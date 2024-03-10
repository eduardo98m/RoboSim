#include "new_collisions.hpp"

ContactManifold colliders_get_contacts(Collider *collider_1, Collider *colider_2)
{
    ContactManifold contacts = ContactManifold();

    scalar penetration;
    vec3 normal;
    // If both colliders are spheres, calling EPA is not only extremely slow, but also provide bad results.
    // GJK is also not necessary. In this case, just calculate everything analytically.
    if (collider_1->type == ColliderType::SPHERE && collider_2->type == ColliderType::SPHERE)
    {
        vec3 distance_vector = collider_1->center - collider_2->center;
        scalar distance_sqd = ti::dot(distance_vector, distance_vector);
        scalar min_distance = collider_1->radius + collider_2->radius;
        if (distance_sqd < (min_distance * min_distance))
        {
            // Spheres are colliding
            normal = ti::normalize(-distance_vector);
            penetration = min_distance - ti::sqrt(distance_sqd);
            clipping_get_contact_manifold(collider_1, colider_2, normal, penetration, contacts);
        }

        return contacts;
    }

    Simplex simplex;

    // Call GJK to check if there is a collision
    if (gjk(collider_1, collider_2, &simplex))
    {
        // There is a collision.

        CollisionResponse response = epa(simplex, collider_1, collider_2);

        // Get the collision normal using EPA
        if (!response.collision)
        {
            return contacts;
        }

        // Finally, clip the results to get the result manifold
        contacts = clipping_get_contact_manifold(collider_1, collider_2, normal, penetration);
    }

    return contacts;
}

ContactManifold clipping_get_contact_manifold(Collider *collider_1, Collider *collider_2, vec3 normal, scalar penetration)
{
    // TODO: For now, we only consider CONVEX and SPHERE colliders.
    // If new colliders are added, we can think about making this more generic.

    ContactManifold contacts = ContactManifold();

    if (collider_1->type == ColliderType::SPHERE)
    {
        vec3 sphere_collision_point = collider_1.find_furthest_point(normal);

        Contact contact;
        contact.collision_point1 = sphere_collision_point;
        contact.collision_point2 = sphere_collision_point - penetration * normal;
        contact.normal = normal;
        contacts.contact_points++;
        contacts[contacts.contact_points] = contact;

        return contacts;
        // array_push(*contacts, contact);
    }
    else if (collider_2->type == ColliderType::SPHERE)
    {

        vec3 sphere_collision_point = collider_2.find_furthest_point(-normal); // support_point(collider_2, -normal);

        Contact contact;
        contact.collision_point1 = sphere_collision_point + penetration * normal;
        contact.collision_point2 = sphere_collision_point;
        contact.normal = normal;
        contacts.contact_points++;
        contacts[contacts.contact_points] = contact;

        return contacts;
        // array_push(*contacts, contact);
    }
    else
    {
        // For now, this case must be convex-convex
        assert(collider_1->type == ColliderType::CONVEX_HULL);
        assert(collider_2->type == ColliderType::CONVEX_HULL);
        contacts = convex_convex_contact_manifold(collider_1, collider_2, normal);

        return contacts;
    }
}

void convex_convex_contact_manifold(Collider *collider_1, Collider *collider_2, vec3 normal, scalar distance)
{

    const scalar EPSILON = 0.0001;

    std::vector<std::pair<size_t, scalar>> contact_faces_1;
    std::vector<std::pair<size_t, scalar>> contact_faces_2;

    for (int i = 0; i < collider_1.faces.size(); i++)
    {
        scalar dirDot = ti::dot(collider_1.face_normals[i], normal);

        if (dirDot > 0.999999)
        {
            scalar posDot = ti::dot(collider_1.compute_face_center(i), normal);

            if (posDot > 0)
            {
                contact_faces_1.push_back({i, dist});
            }
        }
    }

    for (int i = 0; i < collider_2.faces.size(); i++)
    {
        scalar dirDot = ti::dot(collider_2.face_normals[i], normal);

        if (dirDot > 0.999999)
        {
            scalar posDot = ti::dot(collider_2.compute_face_center(i), normal);

            if (posDot > 0)
            {
                contact_faces_2.push_back({i, dist});
            }
        }
    }

    if (contact_faces_1.size() > 0 && contact_faces_2.size > 0)
    {

        /* Face-face clipping algorithm */

        /* Create a quaternion to transform the points to a 2D surface */
        vec3 v = ti::normalize(normal + (vec3){0.0, 0.0, 1.0});
        scalar angle = ti::dot(v, (vec3){0.0, 0.0, 1.0});
        vec3 axis = ti::cross(v, (vec3){0.0, 0.0, 1.0});
        quat rotation_to_2d = quat_from_axis_angle(axis, angle);

        /* Transform the 3D points to the 2D plane (z-coordinate becomes zero) */
        std::vector<vec2> projected_points_1;
        std::vector<vec2> projected_points_2;

        // Sort in descending order based on the second element
        std::sort(contact_faces_1.begin(), contact_faces_1.end(), [](const auto &a, const auto &b)
                  { return b.second < a.second; });
        std::sort(contact_faces_2.begin(), contact_faces_2.end(), [](const auto &a, const auto &b)
                  { return b.second > a.second; });

        for (int j = 0; j < contact_faces_1.size(); j++)
        {

            /* Project 3D points into 2D space */
            for (int vertex_idx : collider_1.faces[contact_faces_1.first])
            {

                vec3 point = ti::rotate(rotation_to_2d, collider_1.verices[vertex_idx]);

                vec2 point_2d = {point.x, point.y};
                bool unique_point = true;

                for (vec2 vertex : projected_points_1)
                {
                    scalar squared_dist = ti::dot(vertex - point_2d, vertex - point_2d);
                    if (squared_dist < 1e-6)
                    {
                        unique_point = false;
                    }
                }

                if (unique_point)
                {
                    projected_points_1.pus_back(point_2d);
                }
            }
        }

        for (int j = 0; j < contact_faces_2.size(); j++)
        {

            /* Project 3D points into 2D space */
            for (int vertex_idx : colliderA.faces[contact_faces_2.first])
            {

                vec3 point = ti::rotate(rotation_to_2d, collider_2.verices[vertex_idx]);

                vec2 point_2d = {point.x, point.y};
                bool unique_point = true;

                for (vec2 vertex : projected_points_2)
                {
                    scalar squared_dist = ti::dot(vertex - point_2d, vertex - point_2d);
                    if (squared_dist < 1e-6)
                    {
                        unique_point = false;
                    }
                }

                if (unique_point)
                {
                    projected_points_2.pus_back(point_2d);
                }
            }
        }

        /* Make sure each 2d face projection is convex */
        // The projection of a set of convex faces should be convex (??)
        // We will skip this by now
        // const gs = new GrahamScan();
        // projectedPoints2D_A = gs.setPoints(projectedPoints2D_A).getHull();
        // projectedPoints2D_B = gs.setPoints(projectedPoints2D_B).getHull();

        /* Perform Sutherland-Hodgman clipping algorithm */
        std::vector<vec2> clipped_Polygon2D = SutherlandHodgmanClipping(projected_points_2, projected_points_1);

        /* Convert back from 2D clip space to 3D world space */
        const inverseTransformMatrix = transformMatrix.clone().invert();

        size_t face_collider_1 = contact_faces_1[0].first;
        size_t face_collider_2 = contact_faces_2[0].first;

        vec3 face_normal_1 = collider_1.face_normals[face_collider_1];
        vec3 face_normal_2 = collider_2.face_normals[face_collider_2];

        vec3 point_1 = collider_1.vertices[collider_1.faces[face_collider_1][0]];
        vec3 point_2 = collider_2.vertices[collider_2.faces[face_collider_2][0]];

        vec3 plane_1_normal = face_normal_1;
        scalar plane_1_offset = ti::dot(face_normal_1, point_1);

        vec3 plane_2_normal = face_normal_2;
        scalar plane_2_offset = ti::dot(face_normal_2, point_2);

        // const clippedPolygon3D_A : Array<Vec3> = []; // Debug only
        // const clippedPolygon3D_B : Array<Vec3> = []; // Debug only

        quat rotation_to_3d = ti::conjugate(rotation_to_2d);
        for (vec2 point_2d : clippedPolygon2D)
        {
            vec3 point_3D = ti::rotate(rotation_to_3d, (vec3){point_2d.x, point_2d.y, 0.0}); //#point2D.x, point2D.y, 0).applyMatrix4(inverseTransformMatrix);

            vec3 point3D_A =  
            const point3D_A = new Vec3();
            const point3D_B = new Vec3();
            planeA.projectPoint(point3DHomogeneous, point3D_A);
            planeB.projectPoint(point3DHomogeneous, point3D_B);

            /* Add contact points to the contact manifold */
            manifold.push([ point3D_A, point3D_B ]);
        }

    }

    /* No face-face contact manifold was found - compute EPA contact point instead */
    if (!manifold.length)
    {

        /* Contact point on the EPA polytope boundary */
        vec3 contact_point = normal * distance;

        /* Compute barycentric coordinates of the contact points on the nearest polytope face */
        vec3 barycentric = this.computeBarycentricCoordinates(contactPoint, minPolygon);

        /* Find the world space contact points on the original shapes */
        vec3 a = minPolygon[0].witnessA* barycentric.x;
        vec3 b = minPolygon[1].witnessA* barycentric.y;
        vec3 c = minPolygon[2].witnessA* barycentric.z;
        vec3 p1 = a+b+c;

        a = minPolygon[0].witnessB* barycentric.x;
        b = minPolygon[1].witnessB* barycentric.y;
        c = minPolygon[2].witnessB* barycentric.z;
        vec3 p2 = a+b+c;

        manifold.push([ p1, p2 ]);
    }

    if (array_length(contacts) == 0)
    {
        // printf("Warning: no intersection was found\n");
    }
}

size_t get_face_with_most_fitting_normal(u32 support_idx, const Collider_Convex_Hull *convex_hull, vec3 normal)
{
    const scalar EPSILON = 0.000001;
    u32 *support_faces = convex_hull->vertex_to_faces[support_idx];

    scalar max_proj = -INFINITY;
    u32 selected_face_idx;
    for (int i = 0; i < array_length(support_faces); ++i)
    {
        Collider_Convex_Hull_Face face = convex_hull->transformed_faces[support_faces[i]];
        scalar proj = gm_vec3_dot(face.normal, normal);
        if (proj > max_proj)
        {
            max_proj = proj;
            selected_face_idx = support_faces[i];
        }
    }

    return selected_face_idx;
}

vec3 compute_barycentric_coordinates(vec3 point, Simplex simplex)
{

    vec3 A = polygon[0];
    vec3 B = polygon[1];
    vec3 C = polygon[2];

    vec3 v0 = B - A;
    vec3 v1 = C - A;
    vec3 v2 = P - A;

    scalar dot00 = ti::dot(v0, v0);
    scalar dot01 = ti::dot(v0, v1);
    scalar dot02 = ti::dot(v0, v2);
    scalar dot11 = ti::dot(v1, v1);
    scalar dot12 = ti::dot(v1, v2);

    scalar denom = dot00 * dot11 - dot01 * dot01;
    scalar v = (dot11 * dot02 - dot01 * dot12) / denom;
    scalar w = (dot00 * dot12 - dot01 * dot02) / denom;
    scalar u = 1.0 - v - w;

    return vec3(u, v, w);
}