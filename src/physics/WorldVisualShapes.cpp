#include "World.hpp"

using namespace robosim;

size_t World::add_visual_shape(VisualShape visual_shape)
{

    this->visual_shapes.push_back(visual_shape);

    return this->visual_shapes.size() - 1;
}

size_t World::attach_mesh_visual_shape(size_t id,
                                      std::string mesh_path,
                                      vec3 position,
                                      quat orientation, 
                                      rs::Color color
                                      )
{

    VisualShape visual_shape = VisualShape{
        .visual_object_path = std::make_optional(mesh_path),
        .pos = position,
        .rot = orientation,
        .color = color,
        .body_id = id};

    size_t vis_shp_id = this->add_visual_shape(visual_shape);

    return vis_shp_id;
}

size_t World::attach_box_visual_shape(size_t id,
                                      vec3 half_extents,
                                      vec3 position,
                                      quat orientation,
                                      rs::Color color)
{

    VisualShape visual_shape = VisualShape{
        .geom = std::make_optional(std::make_shared<hpp::fcl::Box>(
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0)),
        .pos = position,
        .rot = orientation,
        .color = color,
        .body_id = id};

    size_t vis_shp_id = this->add_visual_shape(visual_shape);

    return vis_shp_id;
}

size_t World::attach_sphere_visual_shape(size_t id,
                                         scalar radius,
                                         vec3 position,
                                         quat orientation,
                                         rs::Color color)
{

    VisualShape visual_shape = VisualShape{
        .geom = std::make_optional(std::make_shared<hpp::fcl::Sphere>(radius)),
        .pos = position,
        .rot = orientation,
        .color = color,
        .body_id = id};

    size_t vis_shp_id = this->add_visual_shape(visual_shape);

    return vis_shp_id;
}

size_t World::attach_capsule_visual_shape(size_t id,
                                          scalar radius,
                                          scalar height,
                                          vec3 position,
                                          quat orientation,
                                          rs::Color color)
{

    VisualShape visual_shape = VisualShape{
        .geom = std::make_optional(std::make_shared<hpp::fcl::Capsule>(radius, height)),
        .pos = position,
        .rot = orientation,
        .color = color,
        .body_id = id};

    size_t vis_shp_id = this->add_visual_shape(visual_shape);

    return vis_shp_id;
}

size_t World::attach_cylinder_visual_shape(size_t id,
                                           scalar radius,
                                           scalar height,
                                           vec3 position,
                                           quat orientation,
                                           rs::Color color)
{

    VisualShape visual_shape = VisualShape{
        .geom = std::make_optional(std::make_shared<hpp::fcl::Cylinder>(radius, height)),
        .pos = position,
        .rot = orientation,
        .color = color,
        .body_id = id};

    size_t vis_shp_id = this->add_visual_shape(visual_shape);

    return vis_shp_id;
}

size_t World::attach_plane_visual_shape(size_t id, vec3 normal, scalar offset, rs::Color color)
{
    VisualShape visual_shape = VisualShape{
        .geom = std::make_optional(std::make_shared<hpp::fcl::Halfspace>(normal.x, normal.y, normal.z, offset)),
        // .pos = position,
        // .rot = orientation,
        .color = color,
        .body_id = id};

    size_t vis_shp_id = this->add_visual_shape(visual_shape);

    return vis_shp_id;
}

size_t World::attach_heightmap_visual_shape(size_t id,
                                            scalar x_scale,
                                            scalar y_scale,
                                            std::vector<scalar> heightdata,
                                            size_t x_dims,
                                            size_t y_dims,
                                            rs::Color color)
{

    // Create an Eigen matrix to hold the height data
    hpp::fcl::MatrixXf heightdata_mat(x_dims, y_dims);

    scalar min_value = std::numeric_limits<scalar>::max();

    // Populate the matrix with the provided height data
    for (size_t i = 0; i < x_dims; ++i)
    {
        for (size_t j = 0; j < y_dims; ++j)
        {
            // Assuming the heightdata is stored in row-major order
            heightdata_mat(i, j) = heightdata[i * y_dims + j];
            min_value = ti::min(min_value, heightdata[i * y_dims + j]);
        }
    }

    VisualShape visual_shape = VisualShape{
        .geom = std::make_optional(std::make_shared<hpp::fcl::HeightField<hpp::fcl::AABB>>(x_scale, y_scale, heightdata_mat, min_value)),
        // .pos = position,
        // .rot = orientation,
        .color = color,
        .body_id = id};

    size_t vis_shp_id = this->add_visual_shape(visual_shape);

    return vis_shp_id;
}


std::optional<std::string> World::get_visual_shape_path(int id)
{
    return this->visual_shapes[id].visual_object_path;
}

rs::Color World::get_visual_shape_color(int id)
{
    return this->visual_shapes[id].color;
}

void World::set_visual_shape_color(int id, const rs::Color &color)
{
    this->visual_shapes[id].color = color;
}

void World::set_visual_shape_color(int id, uint8_t r, uint8_t g, uint8_t b, uint8_t alpha)
{
    rs::Color color = {.r = r, .g = g, .b = b, .a = alpha};
    this->set_visual_shape_color(id, color);
}

size_t World::get_number_of_visual_shapes(void){
    return this->visual_shapes.size();
}


std::pair<vec3, quat> World::get_visual_shape_pose(size_t id)
{

    VisualShape *vis_sh = &this->visual_shapes[id];
    Body *body = &this->bodies[vis_sh->body_id];

    quat ori = body->orientation * vis_sh->rot;
    vec3 pos = body->position + ti::rotate(body->orientation, vis_sh->pos);
    

    return std::make_pair(pos, ori);
}

