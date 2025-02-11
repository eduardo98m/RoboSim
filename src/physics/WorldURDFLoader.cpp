#include "World.hpp"
#include <fstream>
using namespace robosim;

size_t World::load_urdf(const std::string &filename, const vec3 &base_position, const quat &base_orientation)
{

    // Open the file and check if it is opened successfully
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open the URDF file: " << filename << "\n";
        return 0; // or handle error as needed
    }

    // Read the file into a string
    std::string urdf_str;
    urdf_str.assign(
        (std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());

    // Close the file after reading
    file.close();

    // Check if the URDF string is empty
    if (urdf_str.empty())
    {
        std::cerr << "Error: The URDF file is empty or could not be read correctly.\n";
        return 0;
    }

    std::shared_ptr<urdf::UrdfModel> model;

    try
    {
        model = urdf::UrdfModel::fromUrdfStr(urdf_str);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught while parsing URDF: " << e.what() << "\n";
        return 0;
    }

    if (model)
    {
        // URDF model loaded successfully
        std::cout << "URDF model loaded successfully!\n";
    }
    else
    {
        // Failed to load URDF model
        std::cerr << "Failed to load URDF model!\n";
        return -1;
    };

    std::map<std::string, size_t> link_name_to_body_id;

    const std::shared_ptr<urdf::Link> &link = model->getRoot();

    // We look for the filepath of the urdf (to get the mesh files locations)
    size_t found = filename.find_last_of("/\\");
    std::string filepath = "";
    if (found != std::string::npos)
    {
        // Extract the substring containing the directory path
        filepath = filename.substr(0, found + 1);
    }
    // We add a base link
    this->add_urdf_link(link, link_name_to_body_id, filepath, &base_position, &base_orientation, true);

    // Iterate over material_map
    std::cout << "\nMaterial Map:\n";
    for (const auto &pair : model->material_map)
    {
        const string &key = pair.first;
        const shared_ptr<urdf::Material> &value = pair.second;
        std::cerr << "Key: " << key << ", Value: " << value << "\n";
    }

    return 0;
}

/**
 *
 */

size_t World::add_urdf_link(const std::shared_ptr<urdf::Link> &link,
                            std::map<std::string, size_t> &link_name_to_body_id,
                            std::string filepath,
                            const vec3 *base_position,
                            const quat *base_orientation,
                            bool root_link)
{

    mat3 intertia = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    scalar mass = 1.0;
    vec3 pos = {0.0, 0.0, 0.0};
    quat ori = (quat){1.0, 0.0, 0.0, 0.0};

    if (link->inertial.has_value())
    {

        mass = link->inertial->mass;

        intertia = {
            {link->inertial->ixx, link->inertial->ixy, link->inertial->ixz},
            {link->inertial->ixy, link->inertial->iyy, link->inertial->iyz},
            {link->inertial->ixz, link->inertial->iyz, link->inertial->izz}};
    }

    pos = (base_position != nullptr) ? (*base_position) : pos;
    ori = (base_orientation != nullptr) ? (*base_orientation) : ori;

    size_t id = this->create_body(
        pos,
        ori,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        mass,
        intertia,
        root_link ? BodyType::STATIC : BodyType::DYNAMIC); // 

    link_name_to_body_id.insert({link->name, id});

    if (!root_link)
    {
        std::string parent_key = link->getParent()->name;

        const shared_ptr<urdf::Joint> &joint = link->parent_joint;
        size_t parent_id = link_name_to_body_id[parent_key];

        vec3 pos = {joint->parent_to_joint_transform.position.x, joint->parent_to_joint_transform.position.y, joint->parent_to_joint_transform.position.z};
        quat ori = {joint->parent_to_joint_transform.rotation.w,
                    joint->parent_to_joint_transform.rotation.x,
                    joint->parent_to_joint_transform.rotation.y,
                    joint->parent_to_joint_transform.rotation.z};

        this->bodies[id]->position = this->bodies[parent_id]->position + ti::rotate(this->bodies[parent_id]->orientation, pos);
        this->bodies[id]->orientation = ori * this->bodies[parent_id]->orientation;

        this->adajacent_links_filter.insert({{id, parent_id}, true});
        this->adajacent_links_filter.insert({{parent_id, id}, true});

        vec3 axis = {joint->axis.x, joint->axis.y, joint->axis.z};
        
        switch (joint->type)
        {
        case urdf::JointType::REVOLUTE:
            {
            // bool limited = joint->limits.has_value();
            // float lower_limit = 0.0;
            // float upper_limit = 0.0;

            // if (limited){
            //     lower_limit = joint->safety.value()->lower_limit;
            //     upper_limit = joint->safety.value()->upper_limit;
            // }

            this->create_revolute_constraint(
                parent_id,
                id,
                axis,
                pos,
                {0.0, 0.0, 0.0},
                1.0e-2,
                1.0e6,
                JointControlType::DRIVEN_BY_POSITION,
                false,
                0.0, 0.0, true);
            break;
            }

        case urdf::JointType::FIXED:
            this->create_fixed_joint_constraint(
                parent_id,
                id,
                pos,
                {0.0, 0.0, 0.0});
            break;
        
        case urdf::JointType::PRISMATIC:
            this->create_prismatic_joint_constraint(
                parent_id,
                id,
                axis,
                pos,
                {0.0, 0.0, 0.0},
                1.0e-2,
                1.0e6,
                JointControlType::FREE,
                false,
                0.0,
                0.0
                );
            break;

        default:
            break;
        }
    }

    for (auto &col : link->collisions)
    {
        shared_ptr<urdf::Geometry> geom = col->geometry.value_or(nullptr);

        vec3 pos = (vec3){col->origin.position.x, col->origin.position.y, col->origin.position.z};
        quat ori = (quat){col->origin.rotation.w, col->origin.rotation.x, col->origin.rotation.y, col->origin.rotation.z};

        switch (geom->type)
        {
        case urdf::GeometryType::SPHERE:
        {
            auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geom);
            this->attach_sphere_collider(id, sphere->radius, pos, ori, 0.5, 0.5, 0.5, false);
            break;
        }

        case urdf::GeometryType::BOX:
        {

            auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
            this->attach_box_collider(id, {box->dim.x * 0.5, box->dim.y * 0.5, box->dim.z * 0.5}, pos, ori, 0.5, 0.5, 0.5, false);
            break;
        }
        case urdf::GeometryType::CYLINDER:
        {
            auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
            this->attach_cylinder_collider(id, cylinder->radius, cylinder->length, pos, ori, 0.5, 0.5, 0.5, false);
            break;
        }
        default:
            break;
        }
    }
    for (auto &vis : link->visuals)
    {
        shared_ptr<urdf::Geometry> geom = vis->geometry.value_or(nullptr);

        vec3 pos = {vis->origin.position.x, vis->origin.position.y, vis->origin.position.z};
        quat ori = (quat){vis->origin.rotation.w,
                          vis->origin.rotation.x,
                          vis->origin.rotation.y,
                          vis->origin.rotation.z};
        
        std::shared_ptr<urdf::Material> material = vis->material.value_or(nullptr);
        rs::Color color;
        if (material)
        {
            color = {
                .r = (u_int8_t)(material->color.r * 255),
                .g = (u_int8_t)(material->color.g * 255),
                .b = (u_int8_t)(material->color.b * 255),
                .a = (u_int8_t)(material->color.a * 255),
            };
        }
        else
        {
            color = {
                .r = 102,
                .g = 102,
                .b = 102,
                .a = 255,
            };
        }

        switch (geom->type)
        {
        case urdf::GeometryType::MESH:
        {
            auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geom);
            vec3 scale = {mesh->scale.x, mesh->scale.y, mesh->scale.z};
            this->attach_mesh_visual_shape(id, filepath + mesh->filename, scale, pos, ori, color);
            break;
        }
        case urdf::GeometryType::SPHERE:
        {
            auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geom);
            this->attach_sphere_visual_shape(id, sphere->radius, pos, ori, color);
            break;
        }

        case urdf::GeometryType::BOX:
        {
            auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
            this->attach_box_visual_shape(id, {box->dim.x * 0.5, box->dim.y * 0.5, box->dim.z * 0.5}, pos, ori, color);
            break;
        }
        case urdf::GeometryType::CYLINDER:
        {
            auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
            this->attach_cylinder_visual_shape(id, cylinder->radius, cylinder->length, pos, ori, color);
            break;
        }
        default:
            break;
        }
    }
    for (const std::shared_ptr<urdf::Link> &child_link : link->child_links)
    {
        this->add_urdf_link(child_link, link_name_to_body_id, filepath);
    }

    return id;
}