#include "Interface.hpp"
#include <cstdio>

Vector3 vec3ToVector3(const vec3 &v)
{
    return {(float)v.x, (float)v.y, (float)v.z};
}
Quaternion quatToQuaternion(const quat &q)
{
    return {(float)q.x, (float)q.y, (float)q.z, (float)q.w};
}


Interface::Interface(std::shared_ptr<robosim::World> world, std::shared_ptr<Visualizer> visualizer)
{
    this->world_ = world;
    this->visualizer_ = visualizer;

    

    this->body_to_visual_shape = {};
    int n_bodies = world_->get_number_of_bodies();

    for (int body_id = 0; body_id < n_bodies; body_id++)
    {
        Vector3 position = vec3ToVector3(this->world_->get_body_position(body_id));
        Quaternion orientation = quatToQuaternion(this->world_->get_body_orientation(body_id));
        int vis_shape_id;
        ShapeInfo collider_info = this->world_->get_collider_info(body_id);

        if (collider_info.type == ShapeType::BOX)
        {
            Box *box = collider_info.box;
            int vis_shape_id = visualizer->add_box(position, orientation, RED, box->half_extents.x, box->half_extents.y, box->half_extents.z);
            this->body_to_visual_shape.push_back({body_id, vis_shape_id});
        }
        else if (collider_info.type == ShapeType::SPHERE)
        {
            Sphere *sphere = collider_info.sphere;
            int vis_shape_id = visualizer->add_sphere(position, orientation, BLUE, sphere->radius);
            this->body_to_visual_shape.push_back({body_id, vis_shape_id});
        }
        else if (collider_info.type == ShapeType::CAPSULE)
        {
            Capsule *capsule = collider_info.capsule;
            // TODO: Create the function to add capsules in the visualizer
            int vis_shape_id = visualizer->add_cylinder(position, orientation, GREEN, capsule->radius, capsule->height);
            this->body_to_visual_shape.push_back({body_id, vis_shape_id});
        }
    }

    int n_revolue_joints = this->world_->get_number_of_revolute_joints();
    for (int i=0; i<n_revolue_joints; i++){
            this->settings.enabled_rev_joints.push_back({i, true});
    }

    auto gui{
    [this, n_revolue_joints](void)
    {
        ImGui::Begin("Visualization Settings");
        ImGui::Text("Revolute Joints");
        ImGui::Separator();
        ImGui::Checkbox("Show Info", &this->settings.show_revolute_joints);
        
        ImGui::SliderInt("IndicatorSize", &this->settings.size, 1, 100);
        if(ImGui::CollapsingHeader("Joints")){
            ImGui::Indent();    
            for (int i=0; i<n_revolue_joints; i++){
                std::string text = "Joint " + std::to_string(i);
                ImGui::Checkbox(text.c_str(), &this->settings.enabled_rev_joints[i].second);
            }
            ImGui::Unindent();
        }
        ImGui::End();
    }
    };

    this->visualizer_->set_imgui_interfaces(gui);
}

void Interface::update(void)
{

    // Bodies
    for (auto pair : this->body_to_visual_shape)
    {
        this->visualizer_->update_visual_object_position_orientation(
            pair.second,
            vec3ToVector3(this->world_->get_body_position(pair.first)),
            quatToQuaternion(this->world_->get_body_orientation(pair.first)));
    }

    // Revolute Joints
    int n_revolue_joints = this->world_->get_number_of_revolute_joints();
    if (this->settings.show_revolute_joints)
    {
        for (auto elem : this->settings.enabled_rev_joints)
        {
            if(!elem.second){continue;}
            RevoluteJointInfo info = this->world_->get_revolute_joint_info(elem.first);

            double arrow_radius = 0.06 * (this->settings.size/100.0);
            double arrow_lenght = 2.0* (this->settings.size/100.0);
            int text_size = 400 * (this->settings.size/100.0);
            this->visualizer_->draw_arrow(
                vec3ToVector3(info.position),
                vec3ToVector3(info.rotation_axis * arrow_lenght),
                arrow_radius,
                VIOLET);

            this->visualizer_->draw_arrow(
                vec3ToVector3(info.position),
                vec3ToVector3(info.body_1_limit_axis* arrow_lenght),
                arrow_radius,
                DARKGREEN);

            this->visualizer_->draw_arrow(
                vec3ToVector3(info.position),
                vec3ToVector3(info.body_2_limit_axis* arrow_lenght),
                arrow_radius,
                ORANGE);

            std::ostringstream angleStream;
            angleStream << std::fixed << std::setprecision(1) << info.current_angle * 180.0 / PI << "°";

            this->visualizer_->draw_text(angleStream.str(), vec3ToVector3(info.position), text_size);
        }
    }
}