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

void Interface::add_collision_object(int body_id, int group_id, Color *color, bool visual_shape)
{   

    std::pair<vec3, quat> pose;
    std::shared_ptr<hpp::fcl::CollisionGeometry> collider_info;
    if (visual_shape){
        pose = this->world_->get_visual_shape_pose(body_id);
        collider_info =  this->world_->get_visual_shape_geometry(body_id);
    }
    else{
        pose = this->world_->get_collider_pose(body_id);
        collider_info =  this->world_->get_collider_geometry(body_id);
    }

    if (!collider_info){return;}
        

    Vector3 position = vec3ToVector3(pose.first);
    Quaternion orientation = quatToQuaternion(pose.second);
    int vis_shape_id = -1;
     

    if (auto box = std::dynamic_pointer_cast<hpp::fcl::Box>(collider_info))
    {
        vis_shape_id = this->visualizer_->add_box(position,
                                                  orientation,
                                                  color ? *color : RED,
                                                  box->halfSide[0] * 2.0, box->halfSide[1] * 2.0, box->halfSide[2] * 2.0,
                                                  group_id);
    }
    else if (auto sphere = std::dynamic_pointer_cast<hpp::fcl::Sphere>(collider_info))
    {
        vis_shape_id = this->visualizer_->add_sphere(position,
                                                     orientation,
                                                     color ? *color : BLUE,
                                                     sphere->radius,
                                                     group_id);
    }
    else if (auto capsule = std::dynamic_pointer_cast<hpp::fcl::Capsule>(collider_info))
    {
        // TODO: Create the function to add capsules in the this->visualizer_
        vis_shape_id = this->visualizer_->add_cylinder(position,
                                                       orientation,
                                                       color ? *color : ORANGE,
                                                       capsule->radius,
                                                       capsule->halfLength * 2.0,
                                                       group_id);
    }
    else if (auto cylinder = std::dynamic_pointer_cast<hpp::fcl::Cylinder>(collider_info))
    {
        vis_shape_id = this->visualizer_->add_cylinder(position,
                                                       orientation,
                                                       color ? *color : PURPLE,
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
                                                    color ? *color : LIGHTGRAY,
                                                    100.0,
                                                    100.0,
                                                    group_id);
    }
    else if (auto heightmap = std::dynamic_pointer_cast<hpp::fcl::HeightField<hpp::fcl::AABB>>(collider_info))
    {

        int x_dim = heightmap->getXDim();
        float x_scale = (float)ti::abs(heightmap->getXGrid()[x_dim - 1] - heightmap->getXGrid()[0]);

        int z_dim = heightmap->getYDim();
        float z_scale = (float)ti::abs(heightmap->getYGrid()[z_dim - 1] - heightmap->getYGrid()[0]);

        position = vec3ToVector3({0.0, 0.0, 0.0});
        orientation = QuaternionFromAxisAngle({0.0, 1.0, 0.0}, 0.0);

        // Convert Eigen::MatrixXd to a linear array of floats
        std::vector<float> mat_vec(heightmap->getHeights().data(), heightmap->getHeights().data() + heightmap->getHeights().size());

        // If you must have a float pointer, use the address of the first element of the vector
        vis_shape_id = this->visualizer_->add_heightmap(position,
                                                        orientation,
                                                        color ? *color : YELLOW,
                                                        heightmap->getXDim(),
                                                        heightmap->getYDim(),
                                                        mat_vec,
                                                        x_scale,
                                                        1.0,
                                                        z_scale,
                                                        group_id);
    }
    else
    {
        return;
    }

    if (group_id == VISUAL_SHAPES_GROUP)
    {
        this->body_to_visual_shape.push_back({body_id, vis_shape_id});
    }
    else if (group_id == COLLISION_SHAPES_GROUP)
    {
        this->collider_to_collision_shape.push_back({body_id, vis_shape_id});
    }
}

void Interface::add_visual_object(int visual_shape_id, int group_id)
{
    std::pair<vec3, quat> pose =  this->world_->get_visual_shape_pose(visual_shape_id);
    Vector3 position = vec3ToVector3(pose.first);
    Quaternion orientation = quatToQuaternion(pose.second);
    int vis_shape_id = -1;

    std::optional<std::string> path = this->world_->get_visual_shape_path(visual_shape_id);
    vec3 scale = this->world_->get_visual_shape_scale(visual_shape_id);

    rs::Color c = this->world_->get_visual_shape_color(visual_shape_id);
    Color color = {static_cast<unsigned char>(c[0]),
                   static_cast<unsigned char>(c[1]),
                   static_cast<unsigned char>(c[2]),
                   static_cast<unsigned char>(c[3])};

    if (path.has_value())
    {
        int vis_shape_id = this->visualizer_->add_mesh(path.value().c_str(), position, orientation, color, scale.x, scale.y, scale.z, VISUAL_SHAPES_GROUP);
        this->body_to_visual_shape.push_back({visual_shape_id, vis_shape_id});
    }
    else
    {
        this->add_collision_object(visual_shape_id, VISUAL_SHAPES_GROUP, &color, true);
    }
}

Interface::Interface(std::shared_ptr<robosim::World> world, std::shared_ptr<Visualizer> visualizer)
{
    this->world_ = world;
    this->visualizer_ = visualizer;

    this->body_to_visual_shape = {};
    this->collider_to_collision_shape = {};
    int n_bodies = world_->get_number_of_bodies();

    int n_colliders = world_->get_number_of_colliders();

    for (int col_id = 0; col_id < n_colliders; col_id++)
    {
        this->add_collision_object(col_id, COLLISION_SHAPES_GROUP, nullptr);
    }


    int n_vis_shapes = world_->get_number_of_visual_shapes();

    for (int vis_shape_id = 0; vis_shape_id < n_vis_shapes; vis_shape_id++)
    {
        this->add_visual_object(vis_shape_id, VISUAL_SHAPES_GROUP);
    }

    int n_revolue_joints = this->world_->get_number_of_revolute_joints();
    for (int i = 0; i < n_revolue_joints; i++)
    {
        this->settings.rev_joints.push_back(std::make_shared<RevoluteJointSettings>((RevoluteJointSettings){i,true,1.0}));
    }

    this->visualizer_->enable_visual_object_group_rendering(VISUAL_SHAPES_GROUP);
    this->visualizer_->disable_visual_object_group_rendering(COLLISION_SHAPES_GROUP);

    int n_prismatic_joints = this->world_->get_number_of_prismatic_joints();
    for (int i = 0; i < n_prismatic_joints; i++)
    {
        this->settings.pris_joints.push_back(std::make_shared<PrismaticJointSettings>((PrismaticJointSettings){i,true,1.0}));
    }

    auto gui{
        [this, n_revolue_joints, n_prismatic_joints](void)
        {
            ImGui::Begin("Visualization Settings");
            ImGui::Checkbox("Show bounding boxes", &this->settings.show_bounding_boxes);

            ImGui::Checkbox("Show AABB tree", &this->settings.show_aabb_tree);

            ImGui::BeginGroup();

            if (ImGui::Button("Toggle visual objects"))
            {
                this->settings.toggle_visual_objects = true; // Set the flag to indicate a reset is requested
            }

            ImGui::SameLine();

            ImGui::Checkbox(" ", &this->settings.render_visual_objects);

            ImGui::EndGroup();
            if (ImGui::Button("Toggle collision objects"))
            {
                this->settings.toggle_collision_shapes = true; // Set the flag to indicate a reset is requested
            }
            ImGui::SameLine();

            ImGui::Checkbox(" ", &this->settings.render_collision_shapes);

            ImGui::Text("Joints");
            ImGui::Separator();
            ImGui::SliderInt("Indicator Size", &this->settings.size, 1, 100);

            ImGui::Text("Contacts");
            ImGui::Separator();
            ImGui::Checkbox("Show Contact Points", &this->settings.show_contact_points);

            ImGui::Separator();

            if (ImGui::CollapsingHeader("Revolute Joints"))
            {
            ImGui::Checkbox("Show Revolute Joints", &this->settings.show_revolute_joints);
                ImGui::Indent();
                for (int i = 0; i < n_revolue_joints; i++)
                {   
                    std::shared_ptr<RevoluteJointSettings> setting = this->settings.rev_joints[i];
                    std::string text = "Revolute Joint " + std::to_string(setting->index);
                    std::string scale_label = "Scale rj " + std::to_string(setting->index);
                    
                    ImGui::Checkbox(text.c_str(), &setting->enabled);
                    ImGui::SliderFloat(scale_label.c_str(), &setting->scale, 0.0, 10.0);
                }
                ImGui::Unindent();
            }

            if (ImGui::CollapsingHeader("Prismatic Joints"))
            {
            ImGui::Checkbox("Show prismatic joints", &this->settings.show_prismatic_joints);
                ImGui::Indent();
                for (int i = 0; i < n_prismatic_joints; i++)
                {
                    std::shared_ptr<PrismaticJointSettings> setting = this->settings.pris_joints[i];
                    std::string text = "Prismatic Joint " + std::to_string(setting->index);
                    std::string scale_label = "Scale pj " + std::to_string(setting->index);
                    ImGui::Checkbox(text.c_str(), &setting->enabled);
                    ImGui::SliderFloat(scale_label.c_str(), &setting->scale, 0.0, 10.0);
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
    if (this->settings.toggle_visual_objects)
    {
        this->settings.toggle_visual_objects = false;
        if (this->settings.render_visual_objects)
        {
            this->visualizer_->disable_visual_object_group_rendering(VISUAL_SHAPES_GROUP);
        }
        else
        {
            this->visualizer_->enable_visual_object_group_rendering(VISUAL_SHAPES_GROUP);
        }
        this->settings.render_visual_objects = !this->settings.render_visual_objects;
    }

    if (this->settings.toggle_collision_shapes)
    {
        this->settings.toggle_collision_shapes = false;
        if (this->settings.render_collision_shapes)
        {
            this->visualizer_->disable_visual_object_group_rendering(COLLISION_SHAPES_GROUP);
        }
        else
        {
            this->visualizer_->enable_visual_object_group_rendering(COLLISION_SHAPES_GROUP);
        }
        this->settings.render_collision_shapes = !this->settings.render_collision_shapes;
    }

    for (auto pair : this->body_to_visual_shape)
    {

        std::pair<vec3, quat> pose = this->world_->get_visual_shape_pose(pair.first);
        this->visualizer_->update_visual_object_position_orientation(
            pair.second,
            vec3ToVector3(pose.first),
            quatToQuaternion(pose.second));

        // if (this->settings.show_bounding_boxes)
        // {
        //     AABB aabb = world_->get_aabb(pair.first);
        //     this->visualizer_->draw_aabb(vec3ToVector3(aabb.min), vec3ToVector3(aabb.max), GREEN);
        // }
    }

    for (auto pair : this->collider_to_collision_shape)
    {
        std::pair<vec3, quat> pose = this->world_->get_collider_pose(pair.first);
        this->visualizer_->update_visual_object_position_orientation(
            pair.second,
            vec3ToVector3(pose.first),
            quatToQuaternion(pose.second));

        // if (this->settings.show_bounding_boxes)
        // {
        //     AABB aabb = world_->get_aabb(pair.first);
        //     this->visualizer_->draw_aabb(vec3ToVector3(aabb.min), vec3ToVector3(aabb.max), GREEN);
        // }
    }

    if (this->settings.show_aabb_tree)
    {
        for (Node *node : this->world_->aabb_tree.nodes)
            this->visualizer_->draw_aabb(vec3ToVector3(node->aabb.min), vec3ToVector3(node->aabb.max), node->is_leaf() ? GREEN : BLUE);
    }

    if (this->settings.show_bounding_boxes)
    {
        for (Node *node : this->world_->aabb_tree.nodes)
            if (node->is_leaf())
            {
                this->visualizer_->draw_aabb(vec3ToVector3(node->aabb.min), vec3ToVector3(node->aabb.max), GREEN);
            }
    }

    // Revolute Joints
    int n_revolue_joints = this->world_->get_number_of_revolute_joints();
    if (this->settings.show_revolute_joints)
    {
        for (shared_ptr<RevoluteJointSettings> joint_setting : this->settings.rev_joints)
        {
            if (!joint_setting->enabled)
            {
                continue;
            }
            RevoluteJointInfo info = this->world_->get_revolute_joint_info(joint_setting->index);

            float scale  = joint_setting->scale * this->settings.size / 50.0;

            double arrow_radius = 0.06 * (scale);
            double arrow_lenght = 2.0 * (scale);
            
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

            float radius = ti::magnitude(info.r_2 - info.r_1);
            
            float upper_lim = 2 * PI;
            float lower_lim = 0.0;
            if (info.limited){
                // This is because how the angles are meassured in the visualizer
                upper_lim = info.upper_limit + PI/2;
                lower_lim = info.lower_limit + PI/2;
            }

            this->visualizer_->draw_ring_section(ti::to_raylib(info.position), 
                                                 ti::to_raylib(info.rotation_axis), 
                                                 radius * 1.05,
                                                 radius * 0.95,
                                                upper_lim,
                                                lower_lim,
                                                 Color{ 120, 121, 241, 255 });
            
            this->visualizer_->draw_ring_section(ti::to_raylib(info.position), 
                                                 ti::to_raylib(info.rotation_axis), 
                                                 radius * 1.07,
                                                 radius * 1.05,
                                                upper_lim,
                                                lower_lim,
                                                 Color{ 90, 90, 255, 255 });
            
            this->visualizer_->draw_ring_section(ti::to_raylib(info.position), 
                                                 ti::to_raylib(info.rotation_axis), 
                                                 radius * 0.95,
                                                 radius * 0.92,
                                                upper_lim,
                                                lower_lim,
                                                 Color{ 90, 90, 255, 255 });
            

            int text_size = 200 * (scale);
            std::ostringstream angleStream;
            angleStream << std::fixed << std::setprecision(1) << info.current_angle * 180.0 / PI << "°";

            this->visualizer_->draw_text(angleStream.str(), vec3ToVector3(info.position), text_size);
        }
    }

    if (this->settings.show_prismatic_joints){
        for (shared_ptr<PrismaticJointSettings> joint_setting : this->settings.pris_joints){
            // Skip the joints that are not enabled
            if (!joint_setting->enabled){continue;}

            float scale  = joint_setting->scale * this->settings.size / 50.0;

            PrismaticJointInfo info = this->world_->get_prismatic_joint_info(joint_setting->index);
            if(info.limited){
                vec3 p_1 = info.position + info.moving_axis * info.lower_limit;
                vec3 p_2 = info.position + info.moving_axis * info.upper_limit;
                this->visualizer_->draw_segment(ti::to_raylib(p_1), 
                                            ti::to_raylib(p_2), 
                                            scale, 
                                            RED);   

            }

            vec3 p_o = info.position;
            vec3 p_f = info.position + info.moving_axis * info.current_position;
            this->visualizer_->draw_segment(ti::to_raylib(p_o), 
                                        ti::to_raylib(p_f), 
                                        scale * 1.05, 
                                        BLUE); 

            std::ostringstream infoStream;
            infoStream << std::fixed << std::setprecision(2) << info.current_position  << "[m]";

            int text_size = 200 * (scale);

            this->visualizer_->draw_text(infoStream.str(), vec3ToVector3(p_f), text_size);

            
        }
    }
    // Contact points:
    if (this->settings.show_contact_points)
    {
        for (const ContactConstraint &contact : this->world_->contact_constraints)
        {
            if (contact.collision)
            {
                this->visualizer_->draw_sphere(vec3ToVector3(contact.p_1), 0.05, RED);
                this->visualizer_->draw_sphere(vec3ToVector3(contact.p_2), 0.05, BLUE);

                this->visualizer_->draw_arrow(
                    vec3ToVector3(contact.p_1),
                    vec3ToVector3(+contact.normal), 0.05, PURPLE);

                this->visualizer_->draw_arrow(
                    vec3ToVector3(contact.p_2),
                    vec3ToVector3(-contact.normal), 0.05, GREEN);
            }
        }
    }
}