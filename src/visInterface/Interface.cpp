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

void Interface::add_collision_object(int body_id, int group_id, Color* color)
{
    Vector3 position = vec3ToVector3(this->world_->get_body_position(body_id));
    Quaternion orientation = quatToQuaternion(this->world_->get_body_orientation(body_id));
    int vis_shape_id = -1;
    std::shared_ptr<hpp::fcl::CollisionGeometry> collider_info = this->world_->get_collider_info(body_id);

    if (auto box = std::dynamic_pointer_cast<hpp::fcl::Box>(collider_info))
    {
        vis_shape_id = this->visualizer_->add_box(position,
                                                      orientation,
                                                      color ? *color:RED,
                                                      box->halfSide[0] * 2.0, box->halfSide[1] * 2.0, box->halfSide[2] * 2.0,
                                                      group_id);
    }
    else if (auto sphere = std::dynamic_pointer_cast<hpp::fcl::Sphere>(collider_info))
    {
        vis_shape_id = this->visualizer_->add_sphere(position,
                                                         orientation,
                                                         color ? *color:BLUE,
                                                         sphere->radius,
                                                         group_id);
    }
    else if (auto capsule = std::dynamic_pointer_cast<hpp::fcl::Capsule>(collider_info))
    {
        // TODO: Create the function to add capsules in the this->visualizer_
        vis_shape_id = this->visualizer_->add_cylinder(position,
                                                           orientation,
                                                           color ? *color:ORANGE,
                                                           capsule->radius,
                                                           capsule->halfLength * 2.0);
    }
    else if (auto cylinder = std::dynamic_pointer_cast<hpp::fcl::Cylinder>(collider_info))
    {
        vis_shape_id = this->visualizer_->add_cylinder(position,
                                                           orientation,
                                                           color ? *color:PURPLE,
                                                           cylinder->radius,
                                                           cylinder->halfLength * 2.0,
                                                           group_id);
    }
    else if (auto plane = std::dynamic_pointer_cast<hpp::fcl::Halfspace>(collider_info))
    {

        float offset = plane->d;
        position = vec3ToVector3(ti::from_eigen(plane->n * plane->d));
        orientation = QuaternionFromVector3ToVector3({0.0, 1.0, 0.0}, vec3ToVector3(ti::from_eigen(plane->n)));
        vis_shape_id = this->visualizer_->add_plane(position,
                                                        orientation,
                                                        color ? *color:LIGHTGRAY,
                                                        100.0,
                                                        100.0,
                                                        group_id);
    }
    else
    {
        return;
    }

    if (group_id == VISUAL_SHAPES_GROUP){
        this->body_to_visual_shape.push_back({body_id, vis_shape_id});
    }else if (group_id == COLLISION_SHAPES_GROUP){
        this->body_to_collision_shape.push_back({body_id, vis_shape_id});
    }
    
}

void Interface::add_visual_object(int body_id, int group_id)
{
    Vector3 position = vec3ToVector3(this->world_->get_body_position(body_id));
    Quaternion orientation = quatToQuaternion(this->world_->get_body_orientation(body_id));
    int vis_shape_id = -1;
    std::shared_ptr<hpp::fcl::CollisionGeometry> collider_info = this->world_->get_collider_info(body_id);

    std::optional<std::string> path = this->world_->get_body_visual_shape_path(body_id);

    
    rs::Color c = this->world_->get_body_color(body_id);
    Color color = {static_cast<unsigned char>(c[0]),
                    static_cast<unsigned char>(c[1]),
                    static_cast<unsigned char>(c[2]),
                    static_cast<unsigned char>(c[3])};
    
    
    if (path.has_value())
    {
        int vis_shape_id = this->visualizer_->add_mesh(path.value().c_str(), position, orientation, color, 1.0, VISUAL_SHAPES_GROUP);
        this->body_to_visual_shape.push_back({body_id, vis_shape_id});
    }
    else
    {
        this->add_collision_object(body_id, VISUAL_SHAPES_GROUP, &color);
    }

    
}

Interface::Interface(std::shared_ptr<robosim::World> world, std::shared_ptr<Visualizer> visualizer)
{
    this->world_ = world;
    this->visualizer_ = visualizer;

    this->body_to_visual_shape = {};
    this->body_to_collision_shape = {};
    int n_bodies = world_->get_number_of_bodies();

    for (int body_id = 0; body_id < n_bodies; body_id++)
    {
        this->add_collision_object(body_id, COLLISION_SHAPES_GROUP, nullptr);
    }

    for (int body_id = 0; body_id < n_bodies; body_id++)
    {
        this->add_visual_object(body_id, VISUAL_SHAPES_GROUP);
    }

    int n_revolue_joints = this->world_->get_number_of_revolute_joints();
    for (int i = 0; i < n_revolue_joints; i++)
    {
        this->settings.enabled_rev_joints.push_back({i, true});
    }

    this->visualizer_->enable_visual_object_group_rendering(VISUAL_SHAPES_GROUP);
    this->visualizer_->disable_visual_object_group_rendering(COLLISION_SHAPES_GROUP);

    auto gui{
        [this, n_revolue_joints](void)
        {
            ImGui::Begin("Visualization Settings");
            ImGui::Checkbox("Show bounding boxes", &this->settings.show_bounding_boxes);

            ImGui::BeginGroup();

            if (ImGui::Button("Toggle visual objects"))
            {
                this->settings.toggle_visual_objects = true; // Set the flag to indicate a reset is requested
            }

            ImGui::SameLine();

            ImGui::Checkbox("Show bounding boxes", &this->settings.render_visual_objects);

            ImGui::EndGroup();
            if (ImGui::Button("Toggle collision objects"))
            {
                this->settings.toggle_collision_shapes = true; // Set the flag to indicate a reset is requested
            }

            ImGui::Text("Revolute Joints");
            ImGui::Separator();
            ImGui::Checkbox("Show Info", &this->settings.show_revolute_joints);
            ImGui::SliderInt("IndicatorSize", &this->settings.size, 1, 100);

            ImGui::Text("Contacts");
            ImGui::Separator();
            ImGui::Checkbox("Show Contact Points", &this->settings.show_contact_points);

            ImGui::Separator();

            if (ImGui::CollapsingHeader("Joints"))
            {
                ImGui::Indent();
                for (int i = 0; i < n_revolue_joints; i++)
                {
                    std::string text = "Joint " + std::to_string(i);
                    ImGui::Checkbox(text.c_str(), &this->settings.enabled_rev_joints[i].second);
                }
                ImGui::Unindent();
            }
            ImGui::End();
        }};

    this->visualizer_->set_imgui_interfaces(gui);
}

void Interface::update(void)
{

    // Bodies
    if (this->settings.toggle_visual_objects){
        this->settings.toggle_visual_objects =false;
        if (this->settings.render_visual_objects){
            this->visualizer_->disable_visual_object_group_rendering(VISUAL_SHAPES_GROUP);
        }else{
            this->visualizer_->enable_visual_object_group_rendering(VISUAL_SHAPES_GROUP);
        }
        this->settings.render_visual_objects= !this->settings.render_visual_objects;
    }

    if (this->settings.toggle_collision_shapes){
        this->settings.toggle_collision_shapes =false;
        if (this->settings.render_collision_shapes){
            this->visualizer_->disable_visual_object_group_rendering(COLLISION_SHAPES_GROUP);
        }else{
            this->visualizer_->enable_visual_object_group_rendering(COLLISION_SHAPES_GROUP);
        }
        this->settings.render_collision_shapes= !this->settings.render_collision_shapes;
    }

    
    for (auto pair : this->body_to_visual_shape)
    {

        this->visualizer_->update_visual_object_position_orientation(
            pair.second,
            vec3ToVector3(this->world_->get_body_position(pair.first)),
            quatToQuaternion(this->world_->get_body_orientation(pair.first)));

        if (this->settings.show_bounding_boxes)
        {
            AABB aabb = world_->get_aabb(pair.first);
            this->visualizer_->draw_aabb(vec3ToVector3(aabb.min), vec3ToVector3(aabb.max), GREEN);
        }
    }
    
    for (auto pair : this->body_to_collision_shape)
    {
        this->visualizer_->update_visual_object_position_orientation(
            pair.second,
            vec3ToVector3(this->world_->get_body_position(pair.first)),
            quatToQuaternion(this->world_->get_body_orientation(pair.first)));

        if (this->settings.show_bounding_boxes)
        {
            AABB aabb = world_->get_aabb(pair.first);
            this->visualizer_->draw_aabb(vec3ToVector3(aabb.min), vec3ToVector3(aabb.max), GREEN);
        }
    }

    // Revolute Joints
    int n_revolue_joints = this->world_->get_number_of_revolute_joints();
    if (this->settings.show_revolute_joints)
    {
        for (auto elem : this->settings.enabled_rev_joints)
        {
            if (!elem.second)
            {
                continue;
            }
            RevoluteJointInfo info = this->world_->get_revolute_joint_info(elem.first);

            double arrow_radius = 0.06 * (this->settings.size / 100.0);
            double arrow_lenght = 2.0 * (this->settings.size / 100.0);
            int text_size = 400 * (this->settings.size / 100.0);
            this->visualizer_->draw_arrow(
                vec3ToVector3(info.position),
                vec3ToVector3(info.rotation_axis * arrow_lenght),
                arrow_radius,
                VIOLET);

            this->visualizer_->draw_arrow(
                vec3ToVector3(info.position),
                vec3ToVector3(info.body_1_limit_axis * arrow_lenght),
                arrow_radius,
                DARKGREEN);

            this->visualizer_->draw_arrow(
                vec3ToVector3(info.position),
                vec3ToVector3(info.body_2_limit_axis * arrow_lenght),
                arrow_radius,
                ORANGE);

            std::ostringstream angleStream;
            angleStream << std::fixed << std::setprecision(1) << info.current_angle * 180.0 / PI << "°";

            this->visualizer_->draw_text(angleStream.str(), vec3ToVector3(info.position), text_size);
        }
    }
    // Contact points:
    if (this->settings.show_contact_points)
    {
        for (const ContactConstraint &contact : this->world_->contact_constraints)
        {
            if (contact.collision)
            {
                this->visualizer_->draw_sphere(vec3ToVector3(contact.collision_response.contact_point_1), 0.05, RED);
                this->visualizer_->draw_sphere(vec3ToVector3(contact.collision_response.contact_point_2), 0.05, BLUE);

                this->visualizer_->draw_arrow(
                    vec3ToVector3(contact.collision_response.contact_point_1),
                    vec3ToVector3(+contact.collision_response.normal), 0.05, PURPLE);

                this->visualizer_->draw_arrow(
                    vec3ToVector3(contact.collision_response.contact_point_2),
                    vec3ToVector3(-contact.collision_response.normal), 0.05, GREEN);
            }
        }
    }
}