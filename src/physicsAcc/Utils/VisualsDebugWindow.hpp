
#include "physicsAcc/API/World.hpp"

struct VisualsDebugWindow
{

  void render(World &world)
  {

    // Don't render anything if there is no visualizer
    if (!world.visualizer)
      return;

    ImGui::Begin("Visuals");

    if (ImGui::CollapsingHeader("Bodies"))
    {
      for (size_t body_id = 0; body_id < world.bodies.n_bodies; ++body_id)
      {
        rbvs::VisualModel &vm =
            world.visualizer->visualizer.entity_manager
                .getComponent<rbvs::VisualModel>(
                    world.visualizer->body_to_frame[body_id]);

        std::string visible_label = "Body -" + std::to_string(body_id) + " :Visible";
        std::string color_label = "Body -" + std::to_string(body_id) + " :Color";
        std::string scale_label = "Body -" + std::to_string(body_id) + " : Scale";
        ImGui::Checkbox(visible_label.c_str(), &vm.visible);
        float color[4] = {vm.color.r / 255.0f, vm.color.g / 255.0f,
                          vm.color.b / 255.0f, vm.color.a / 255.0f};
        if (ImGui::ColorEdit4(color_label.c_str(), color))
        {
          vm.color.r = static_cast<unsigned char>(color[0] * 255.0f);
          vm.color.g = static_cast<unsigned char>(color[1] * 255.0f);
          vm.color.b = static_cast<unsigned char>(color[2] * 255.0f);
          vm.color.a = static_cast<unsigned char>(color[3] * 255.0f);
        }
        
        float scale = vm.scale.x; // We only change the scale here + We force a uniform scale
        if (ImGui::DragFloat(scale_label.c_str(), &scale, 0.01f, 0.0f, 10000.0f))
        {
          vm.scale.x = scale;
          vm.scale.y = scale;
          vm.scale.z = scale;
        }
      }
    }

    if (ImGui::CollapsingHeader("Colliders"))
    {
      for (size_t collider_id = 0; collider_id < world.colliders.n_colliders;
           ++collider_id)
      {
        rbvs::VisualModel &vm =
            world.visualizer->visualizer.entity_manager
                .getComponent<rbvs::VisualModel>(
                    world.visualizer->collider_to_vis_shape[collider_id]);

        std::string visible_label = "Collider -" + std::to_string(collider_id) + " :Visible";
        std::string color_label = "Collider -" + std::to_string(collider_id) + " :Color";

        ImGui::Checkbox(visible_label.c_str(), &vm.visible);
        float color[4] = {vm.color.r / 255.0f, vm.color.g / 255.0f,
                          vm.color.b / 255.0f, vm.color.a / 255.0f};
        if (ImGui::ColorEdit4(color_label.c_str(), color))
        {
          vm.color.r = static_cast<unsigned char>(color[0] * 255.0f);
          vm.color.g = static_cast<unsigned char>(color[1] * 255.0f);
          vm.color.b = static_cast<unsigned char>(color[2] * 255.0f);
          vm.color.a = static_cast<unsigned char>(color[3] * 255.0f);
        }
      }
    }

    ImGui::End();
  }
};
