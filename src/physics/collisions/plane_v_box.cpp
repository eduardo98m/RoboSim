#include "collisions.hpp"

CollisionResponse compute_collision_response(const vec3 &box_position, const quat &box_orientation, const Box &box, Plane plane)
{

    CollisionResponse response;

    response.normal = plane.normal;

    scalar extent = ti::dot(box.half_extents, ti::abs(ti::rotate(box_orientation, plane.normal)));

    scalar distance = ti::dot(box_position, plane.normal) + plane.offset;

    std::vector<vec3> vertices = get_box_vertices(box.half_extents, box_position, box_orientation);

    vec3 contact_point = vertices[0];
    //scalar min_dist = ti::dot(contact_point, plane.normal) + plane.offset;

    // std::cerr << "position : " << box_position << "\n";
    // for (const vec3 &v : vertices)
    // {
    //     scalar dist = ti::dot(v, plane.normal) + plane.offset;

    //     std::cerr << "v : " << v  << dist << "\n";
    //     if (dist < min_dist)
    //     {
    //         contact_point = v;
    //         min_dist = dist;
    //     }
    // }

    std::vector<vec3> contact_points;
    scalar threshold = 1e-3;
    scalar min_dist = INFINITY;

    for (const vec3 &v : vertices)
    {
        scalar dist = ti::dot(v, plane.normal) + plane.offset;
        if (dist < min_dist - threshold) // If a point is significantly closer
        {
            contact_points.clear();      // Clear the previous points
            contact_points.push_back(v); // Start a new list
            min_dist = dist;             // Update the minimum distance
        }
        else if ((min_dist - threshold) <= dist && dist <= (min_dist + threshold)) // If the point is within the threshold
        {
            contact_points.push_back(v); // Add it to the list
        }
    }

    vec3 average_contact_point(0, 0, 0);
    for (const vec3 &v : contact_points)
    {
        average_contact_point += v;
    }
    average_contact_point /= contact_points.size();

    // Set contact points
    response.contact_point_1 = average_contact_point;
    response.contact_point_2 = average_contact_point - plane.normal * min_dist;

    // // Set contact points
    // response.contact_point_1 = contact_point;
    // response.contact_point_2 = contact_point - plane.normal * min_dist;

    std::cerr << "contact_point_1 : " << response.contact_point_1 << "\n";
    std::cerr << "contact_point_2 : " << response.contact_point_2 << "\n";
    std::cerr << "distance : " << min_dist << "\n";

    // response.contact_point_1 = box_position - plane.normal * extent;//local_contact_1;

    // response.contact_point_2 = box_position - plane.normal * distance;//ti::rotate(box_orientation, local_contact_2);

    // std::cerr << "contact_point_1 : " << response.contact_point_1  << "\n";
    // std::cerr << "contact_point_2 : " << response.contact_point_2  << "\n";

    std::cerr << "------------------------------\n";

    return response;
}