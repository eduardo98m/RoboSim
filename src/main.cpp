
#include "physics/math/math.hpp"
#include "physics/constraints/RevoluteJointConstraint.hpp"

#include "scenarios/collisions_scenario.hpp"
#include "scenarios/robot_arm_scenario.hpp"

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

    enum ScenarioType
    {
        COLLISIONS,
        ROBOT
    };

    std::vector<std::string> scenario_names = {"Collisions Scenario", "Robot Scenario"};
    int selected_scenario = ScenarioType::ROBOT; // Default to collisions scenario

    auto gui_interface{
        [&reset_scenario, &selected_scenario, scenario_names](void)
        {
            ImGui::Begin("Scenario controller");
            ImGui::Separator();
            if (ImGui::Button("Reset scenario"))
            {
                reset_scenario = true; // Set the flag to indicate a reset is requested
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
        }
    };

    robosim::World world = collisions_scenario();

    Visualizer visualizer(1208, 720, "RoboVis");
    visualizer.set_up_camera();

    visualizer.set_imgui_interfaces(gui_interface);

    std::shared_ptr<robosim::World> world_ptr(&world);
    std::shared_ptr<Visualizer> vis_ptr(&visualizer);
    Interface* interface = new Interface(world_ptr, vis_ptr);
    

    while (!WindowShouldClose())
    {
        visualizer.update();
        world.step();
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
            
            default:
                break;
            }
            visualizer.clear_visual_objects();
            visualizer.clear_gui_interfaces();
            visualizer.set_imgui_interfaces(gui_interface);

            delete interface; // This is to avoid a memory leak
            interface = new Interface(world_ptr, vis_ptr);

            reset_scenario = false; // Reset the flag
        }
    }

    // De-initialize
    CloseWindow();

    return 0;
}