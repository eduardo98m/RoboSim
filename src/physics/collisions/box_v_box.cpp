#include "collisions.hpp"

std::vector<vec3> get_axes(const quat &orientation_1, const quat &orientation_2)
{
    std::vector<vec3> axes(15);
    // axes.reserve(15);

    // Box Faces of the first box
    mat3 axes_1 = ti::mat3_cast(orientation_1);
    // Box Faces of the second box
    mat3 axes_2 = ti::mat3_cast(orientation_2);
    ;

    for (int i = 0; i < 3; i++)
    {

        axes[i] = axes_1[i];
    }

    for (int i = 0; i < 3; i++)
    {

        axes[i + 3] = axes_2[i];
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            vec3 ax = ti::cross(axes_1[i], axes_2[j]);
            if (ti::magnitude(ax) > EPSILON)
            {
                axes[6 + i + j] = ti::normalize(ax);
            }
        }
    }

    return axes;
}

ContactPoint compute_collision_response(const vec3 &position_1, const quat &orientation_1, const vec3 &position_2, const quat &orientation_2, const Box &box_1, const Box &box_2)
{

    ContactPoint response = ContactPoint();

    std::vector<vec3> axes = get_axes(orientation_1, orientation_2);

    std::vector<vec3> vertices_1 = get_box_vertices(box_1.half_extents, position_1, orientation_1);
    std::vector<vec3> vertices_2 = get_box_vertices(box_2.half_extents, position_2, orientation_2);


    response.normal = {0.0, 1.0, 0.0};
    response.contact_point_1 = {0.0, -1.0, 0.0};
    response.contact_point_2 = {0.0, 0.0, 0.0};

    bool collision = false;
    scalar collision_direction = 1.0;

    scalar min_overlap = INFINITY;
    size_t n_points = 0;
    std::vector<vec3> normals;
    std::vector<vec3> contact_points_1;
    std::vector<vec3> contact_points_2;
    std::vector<scalar> penetration;

    for (const vec3 &axis : axes)
    {

        if (ti::magnitude(axis) == 0.0){
            continue;
        }
        vec2 projections_1, projections_2;
        vec3 min_vertex_1, max_vertex_1;
        vec3 min_vertex_2, max_vertex_2;

        std::tie(projections_1, min_vertex_1, max_vertex_1) = get_vertices_projection_max_and_min(vertices_1, axis);
        std::tie(projections_2, min_vertex_2, max_vertex_2) = get_vertices_projection_max_and_min(vertices_2, axis);

        // Check if the projections overlap
        std::pair<scalar, scalar> overlap_info = get_projections_overlap(projections_1[0],
                                                                         projections_1[1],
                                                                         projections_2[0],
                                                                         projections_2[1]);
        scalar overlap = overlap_info.first;
        scalar direction = overlap_info.second;

        if (overlap <= 0)
        {
            collision = false;
            response.normal = {0.0, 1.0, 0.0};
            response.contact_point_1 = {0.0, -1.0, 0.0};
            response.contact_point_2 = {0.0, 0.0, 0.0};
            break;
        }
        else if (overlap < min_overlap)
        {
            // TODO : Check order
            // Check if the minimum projection vertex of box 1 is within the overlap range
            if (direction > 0.0)
            {
                contact_points_1.push_back(max_vertex_1);
                contact_points_2.push_back(min_vertex_2);
                response.contact_point_1 =max_vertex_1;
                response.contact_point_2 =min_vertex_2;
            }
            else {
                contact_points_1.push_back(min_vertex_1);
                contact_points_2.push_back(max_vertex_2);
                response.contact_point_1 =min_vertex_1;
                response.contact_point_2 =max_vertex_2;
            }


            n_points++;

            min_overlap = ti::min(overlap, min_overlap);
            normals.push_back(direction * axis);
            penetration.push_back(overlap);
            response.normal = direction * axis;
            
        }
    }

    // if (n_points > 0)
    // {
    //     vec3 p_1 = {0.0, 0.0, 0.0};
    //     vec3 p_2 = {0.0, 0.0, 0.0};
    //     vec3 normal = {0.0, 0.0, 0.0};
    //     scalar total_penetration = 0.0;

    //     for (int i = 0; i < n_points; i++)
    //     {
    //         if (ti::abs(penetration[i]) <= (ti::abs(min_overlap) + 0.001))
    //         {


    //             // Use these vertices as the contact points
    //             p_1 += contact_points_1[i] * penetration[i];
    //             p_2 += contact_points_2[i] * penetration[i];
    //             normal += normals[i] * penetration[i];
    //             total_penetration += penetration[i];
    //         }
    //     }

    //     if (total_penetration > 0.0)
    //     {
    //         p_1 /= total_penetration;
    //         p_2 /= total_penetration;
    //         normal /= total_penetration;
    //         response.contact_point_1 = p_1;
    //         response.contact_point_2 = p_2;
    //         response.normal = ti::normalize(normal);
    //         response.contact_point_1 = p_1;
    //         response.contact_point_2 = p_2;
    //     }
        
    // }

    
    response.normal = ti::normalize(response.normal);

    return response;
}
