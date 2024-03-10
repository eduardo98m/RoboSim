#include "collisions.hpp"

ContactPoint  compute_collision_response(const vec3& box_position, const quat&  box_orientation, const vec3& sphere_position, const Box &box, const Sphere & sphere)
{

    ContactPoint response = ContactPoint();

    mat3 box_face_normals = ti::mat3_cast(box_orientation);
    std::vector<vec3> axes = {
        (vec3){box_face_normals[0][0], box_face_normals[0][1], box_face_normals[0][2]},
        (vec3){box_face_normals[1][0], box_face_normals[1][1], box_face_normals[1][2]},
        (vec3){box_face_normals[2][0], box_face_normals[2][1], box_face_normals[2][2]},
        ti::normalize(box_position - sphere_position)
    } ;

    std::vector<vec3> vertices = get_box_vertices(box.half_extents, box_position, box_orientation);
    
    response.normal = {0.0, 1.0, 0.0};
    response.contact_point_1 = {0.0, -1.0, 0.0};
    response.contact_point_2 = {0.0, 0.0, 0.0};

    bool collision = false;
    scalar collision_direction = 1.0;

    scalar max_overlap = -INFINITY;
    for (const vec3 &axis : axes)
    {
        vec2 projections_box;
        vec3 min_vertex, max_vertex;
        std::tie(projections_box, min_vertex, max_vertex)= get_vertices_projection_max_and_min(vertices, axis);

        scalar projection_sphere_min = ti::dot(sphere_position - axis * sphere.radius, axis);
        scalar projection_sphere_max = ti::dot(sphere_position + axis * sphere.radius, axis);

        // Check if the projections overlap
        std::pair<scalar, scalar> overlap_info = get_projections_overlap(projections_box[0],
                                                                         projections_box[1],
                                                                         projection_sphere_min,
                                                                         projection_sphere_max);
        scalar overlap = overlap_info.first;
        scalar direction = overlap_info.second;

        if (overlap <= 0){
            collision = false;
            response.normal = {0.0, 1.0, 0.0};
            response.contact_point_1 = {0.0, -1.0, 0.0};
            response.contact_point_2 = {0.0, 0.0, 0.0};
            break;
        } 
        else if (overlap > max_overlap){// TODO: Check this condition
            max_overlap = overlap;
            collision = true;
            collision_direction = direction;
        }

        if (collision){
            // TODO : Check order  
            response.normal = -axis; 
            scalar box_extent = ti::dot(box.half_extents, ti::abs(ti::rotate(box_orientation, response.normal)));
            response.contact_point_1 = box_position + response.normal * box_extent ;
            response.contact_point_2 = sphere_position - response.normal * sphere.radius;
        }
    }


    return response;
}