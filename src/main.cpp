
#include "physics/math/math.hpp"
#include "physics/constraints/RevoluteJointConstraint.hpp"

#include "scenarios/collisions_scenario.hpp"
#include "scenarios/robot_arm_scenario.hpp"
#include "scenarios/simple_box_collisions_scenario.hpp"
#include "scenarios/collisions_groups_scenario.hpp"
// Import the sine function from the standard library
#include <math.h>
#include <iostream>
#include "Visualizer.hpp"
#include <raylib.h>
#include "physics/World.hpp"
#include "Interface.hpp"
#include <memory>
#include <random>
#include "utils/Logging.hpp"

int main(int argc, char *argv[])
{

    bool reset_scenario = false;
    bool pause_simulation = false;

    enum ScenarioType
    {
        COLLISIONS,
        ROBOT,
        BOX_COLLISIONS,
        COLLISION_GROUPS
    };

    std::vector<std::string> scenario_names = {"Collisions Scenario", "Robot Scenario", "Box Collisions", "Collision groups"};

    int selected_scenario = ScenarioType::BOX_COLLISIONS; // Default to collisions scenario

    auto gui_interface{
        [&pause_simulation, &reset_scenario, &selected_scenario, scenario_names](void)
        {
            ImGui::Begin("Scenario controller");
            ImGui::Separator();
            if (ImGui::Button("Reset scenario"))
            {
                reset_scenario = true; // Set the flag to indicate a reset is requested
            }
            if (ImGui::Button("Pause"))
            {
                pause_simulation = !pause_simulation; // Set the flag to indicate a reset is requested
            }
            ImGui::Separator();
            if (ImGui::BeginCombo("Select Scenario", scenario_names[selected_scenario].c_str()))
            {
                for (int i = 0; i < scenario_names.size(); ++i)
                {
                    bool is_selected = (selected_scenario == i);
                    if (ImGui::Selectable(scenario_names[i].c_str(), is_selected))
                    {
                        selected_scenario = i;
                    }
                    if (is_selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::End();

            // Draw square and points
            ImGui::Begin("2D Drawing Example");

            // Draw the box
            ImVec2 canvasSize(200, 200);
            ImVec2 canvasPos = ImGui::GetCursorScreenPos();
            ImDrawList *drawList = ImGui::GetWindowDrawList();
            drawList->AddRect(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y), IM_COL32(255, 0, 0, 255));

            // Draw some points
            drawList->AddCircleFilled(ImVec2(canvasPos.x + canvasSize.x / 2, canvasPos.y + canvasSize.y / 2), 5, IM_COL32(0, 255, 0, 255));
            drawList->AddCircleFilled(ImVec2(canvasPos.x + canvasSize.x / 3, canvasPos.y + canvasSize.y / 3), 5, IM_COL32(0, 0, 255, 255));
            drawList->AddCircleFilled(ImVec2(canvasPos.x + canvasSize.x / 2 + 50, canvasPos.y + canvasSize.y / 2 - 50), 5, IM_COL32(255, 165, 0, 255));

            ImGui::End();
        }};

    robosim::World world = simple_box_collisions_scenario();
    ;

    Visualizer visualizer(1920, 1080, "RoboVis");
    visualizer.set_up_camera();

    visualizer.load_shader(0, "../src/RoboVis/shaders/hybrid_raymarch.fs", 0);
    // visualizer.load_shader(0, "../src/RoboVis/shaders/hybrid_raster.fs", 0);

    visualizer.set_imgui_interfaces(gui_interface);

    std::shared_ptr<robosim::World> world_ptr(&world);
    std::shared_ptr<Visualizer> vis_ptr(&visualizer);
    Interface *interface = new Interface(world_ptr, vis_ptr);
    //

    while (!WindowShouldClose())
    {
        visualizer.update();

        if (!pause_simulation)
        {
            world.step();
        }

        interface->update();

        if (reset_scenario)
        {
            MSG("Reset Scenario\n");
            // Reset the world here

            switch (selected_scenario)
            {
            case ScenarioType::COLLISIONS:
                world = collisions_scenario();
                break;

            case ScenarioType::ROBOT:
                world = robot_arm_scenario();
                break;

            case ScenarioType::BOX_COLLISIONS:
                world = simple_box_collisions_scenario();
                break;
            case ScenarioType::COLLISION_GROUPS:
                world = collision_groups_scenario();
            default:
                break;
            }
            visualizer.clear_visual_objects();
            visualizer.clear_gui_interfaces();
            visualizer.unload_models();
            visualizer.set_imgui_interfaces(gui_interface);
            world.step();

            delete interface; // This is to avoid a memory leak
            interface = new Interface(world_ptr, vis_ptr);

            reset_scenario = false; // Reset the flag
        }
    }

    // De-initialize
    CloseWindow();

    return 0;
}