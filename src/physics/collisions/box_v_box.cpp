#include "collisions.hpp"

std::vector<vec3> get_axes(const quat &orientation_1, const quat &orientation_2)
{
    std::vector<vec3> axes(15);
    //axes.reserve(15);

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
            
        axes[i+3] = axes_2[i];
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

CollisionResponse compute_collision_response(const vec3 &position_1, const quat &orientation_1, const vec3 &position_2, const quat &orientation_2, const Box &box_1, const Box &box_2)
{

    CollisionResponse response = CollisionResponse();

    std::vector<vec3> axes = get_axes(orientation_1, orientation_2);

    std::vector<vec3> vertices_1 = get_box_vertices(box_1.half_extents, position_1, orientation_1);
    std::vector<vec3> vertices_2 = get_box_vertices(box_2.half_extents, position_2, orientation_2);

    response.normal = {0.0, 1.0, 0.0};
    response.contact_point_1 = {0.0, -1.0, 0.0};
    response.contact_point_2 = {0.0, 0.0, 0.0};

    bool collision = false;
    scalar collision_direction = 1.0;

    scalar min_overlap = INFINITY;
    for (const vec3 &axis : axes)
    {
        std::pair<scalar, scalar> projections_1 = get_vertices_projection_max_and_min(vertices_1, axis);
        std::pair<scalar, scalar> projections_2 = get_vertices_projection_max_and_min(vertices_2, axis);

        // Check if the projections overlap
        std::pair<scalar, scalar> overlap_info = get_projections_overlap(projections_1.first,
                                                                         projections_1.second,
                                                                         projections_2.first,
                                                                         projections_2.second);
        scalar overlap = overlap_info.first;
        scalar direction = overlap_info.second;

        if (overlap <= 0)
        {
            collision = false;
            break;
        }
        else if (overlap < min_overlap)
        { // TODO: Check this condition
            min_overlap = overlap;
            response.normal = axis;
            collision = true;
            collision_direction = direction;
        }

        if (collision)
        {
            scalar extent_1 = ti::dot(box_1.half_extents, ti::abs(ti::rotate(orientation_1, response.normal)));
            scalar extent_2 = ti::dot(box_2.half_extents, ti::abs(ti::rotate(orientation_2, response.normal)));
            response.contact_point_1 = position_1 - response.normal * extent_1;
            response.contact_point_2 = position_2 + response.normal * extent_2;
        }
    }

    return response;
}
