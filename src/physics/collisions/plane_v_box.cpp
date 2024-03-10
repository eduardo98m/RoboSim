#include "collisions.hpp"

ContactPoint compute_collision_response(const vec3 &box_position, const quat &box_orientation, const Box &box, Plane plane)
{

    ContactPoint response;

    std::vector<vec3> vertices = get_box_vertices(box.half_extents, box_position, box_orientation);
    std::vector<vec3> contact_points_1;
    std::vector<vec3> contact_points_2;
    std::vector<scalar> penetration;

    response.normal =  plane.normal;


    size_t n_points = 0;
    for (const vec3 &v : vertices)
    {
        scalar dist = ti::dot(v, plane.normal) + plane.offset;

        if (dist < 0){
            contact_points_1.push_back(v);
            contact_points_2.push_back(v - plane.normal * dist);
            penetration.push_back(ti::abs(dist));
            n_points++;
            
        }        
    }

    vec3 p_1 = {0.0, 0.0, 0.0};
    vec3 p_2 = {0.0, 0.0, 0.0};
    if (n_points > 0){
        scalar total_presentation = 0.0;
        for (int i =0 ; i < n_points; i++){
            p_1 += contact_points_1[i] * penetration[i];
            p_2 += contact_points_2[i] * penetration[i];
            total_presentation += penetration[i];
        }
        p_1 = p_1/total_presentation;
        p_2 = p_2/total_presentation;
    }
    

    response.contact_point_1 = p_1;
    response.contact_point_2 = p_2; 

    return response;
}