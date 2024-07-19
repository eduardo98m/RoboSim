#include "World.hpp"
#include "urdf/model.h"
#include "urdf/link.h"
#include "urdf/joint.h"
#include <fstream>
using namespace robosim;

size_t World::load_urdf(const std::string &filename, vec3 base_position)
{

    std::ifstream file(filename);
    std::string urdf_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());


    size_t found = filename.find_last_of("/\\");
    std::string filepath = "";
    if (found != std::string::npos) {
        // Extract the substring containing the directory path
        filepath =  filename.substr(0, found + 1);
    }

    std::shared_ptr<urdf::UrdfModel> model;

    model = urdf::UrdfModel::fromUrdfStr(std::string(urdf_str));

    if (model)
    {
        // URDF model loaded successfully
        std::cout << "URDF model loaded successfully!\n";
    }
    else
    {
        // Failed to load URDF model
        std::cerr << "Failed to load URDF model!\n";
        return 1;
    };

    std::map<std::string, size_t> link_name_to_body_id;

    // Iterate over link_map
    cout << "Link Map:\n";
    for (const auto &pair : model->link_map)
    {
        const string &key = pair.first;
        const shared_ptr<urdf::Link> &value = pair.second;

        mat3 intertia = {
            {value->inertial->ixx, value->inertial->ixy, value->inertial->ixz},
            {value->inertial->ixy, value->inertial->iyy, value->inertial->iyz},
            {value->inertial->ixz, value->inertial->iyz, value->inertial->izz}};

        size_t id = this->create_body(
            base_position,
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            value->inertial->mass,
            intertia,
            BodyType::DYNAMIC);

        for (auto &col : value->collisions)
        {
            shared_ptr<urdf::Geometry> geom = col->geometry.value_or(nullptr);

            switch (geom->type)
            {
            case urdf::GeometryType::SPHERE:
            {
                auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geom);
                this->attach_sphere_collider(id, sphere->radius);
                break;
            }

            case urdf::GeometryType::BOX:
            {

                auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
                this->attach_box_collider(id, {box->dim.x,
                                                 box->dim.y,
                                                 box->dim.z});
                break;
            }
            case urdf::GeometryType::CYLINDER:
            {
                auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
                this->attach_cylinder_collider(id, cylinder->radius, cylinder->length);
                break;
            }
            default:
                break;
            }
        }
        for (auto &vis : value->visuals)
        {
            shared_ptr<urdf::Geometry> geom = vis->geometry.value_or(nullptr);

            switch (geom->type)
            {
            case urdf::GeometryType::MESH:
            {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geom);
                this->set_body_visual_shape_path(id, filepath + mesh->filename);
                std::shared_ptr<urdf::Material> material  = vis->material.value_or(nullptr);
                rs::Color color =  {
                    .r = material->color.r * 255,
                    .g = material->color.g * 255,
                    .b = material->color.b * 255,
                    .a = material->color.a * 255,
                };
                this->set_body_color(id, color);
                break;
            }
            default:
                break;
            }
        }
    }

    // Iterate over joint_map
    cout << "\nJoint Map:\n";
    for (const auto &pair : model->joint_map)
    {
        const string &key = pair.first;
        const shared_ptr<urdf::Joint> &value = pair.second;
        cout << "Key: " << key << ", Value: " << value << endl;
    }

    // Iterate over material_map
    cout << "\nMaterial Map:\n";
    for (const auto &pair : model->material_map)
    {
        const string &key = pair.first;
        const shared_ptr<urdf::Material> &value = pair.second;
        cout << "Key: " << key << ", Value: " << value << endl;
    }

    return 0;
}