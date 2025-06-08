#pragma once

// STD
#include <vector>
#include <string>
#include <functional>
#include <algorithm> // Para std::remove_if
#include <sstream>   // Para std::stringstream
// IMGUI
#include "imgui.h"   
// Internal
#include "physicsAcc/API/World.hpp" // Para acceder a la instancia de World
#include "physicsAcc/Utils/PrintUtils.hpp" // Para get_generic_info

enum class DebugEntityType {
    NONE = 0, // Importante que NONE sea 0 para el ImGui::Combo
    BODY,
    CONSTRAINT,
    JOINT,
    // Puedes añadir más si lo necesitas, como CONTACT o COLLIDER
};

// Estructura para el estado de cada ventana de depuración
struct DebugWindow {
    std::string name; // Nombre único para la ventana ImGui
    DebugEntityType selected_type = DebugEntityType::NONE;
    int entity_index = 0; // Usamos int para ImGui::InputInt
    bool is_open = true; // Para permitir cerrar la ventana
    bool is_paused = false; // ¡Nuevo! Estado de pausa para esta ventana
    std::string cached_data; // ¡Nuevo! Para almacenar la última información cuando está pausada
};

// Clase para manejar múltiples ventanas de depuración
class DebugGUIHandler {
public:
    DebugGUIHandler() = default; // Constructor por defecto

    // Este es el método principal que se llamará desde la lambda del Visualizer
    void render_debug_uis(World& world) {
        // Panel de control principal
        ImGui::Begin("Debug Control Panel");
        if (ImGui::Button("Add New Debug Window")) {
            add_debug_window("Debug Window " + std::to_string(next_window_id++));
        }

        ImGui::SameLine();
        // Botón de pausa global
        if (ImGui::Checkbox("Pause All Debuggers", &global_pause_active)) {
            // Cuando la pausa global cambia, actualiza todas las ventanas
            for (auto& window : debug_windows) {
                window.is_paused = global_pause_active;
                if (global_pause_active) {
                    // Al pausar, cachea la información actual de todas las ventanas
                    cache_window_data(window, world);
                }
            }
        }
        ImGui::Text("Global Pause Status: %s", global_pause_active ? "PAUSED" : "RUNNING");

        ImGui::End();

        // Luego, renderiza todas las ventanas de depuración activas
        for (auto& window : debug_windows) {
            if (window.is_open) {
                render_single_debug_window(window, world); // Pasa world a la función de renderizado
            }
        }
        // Limpiar ventanas cerradas
        debug_windows.erase(
            std::remove_if(debug_windows.begin(), debug_windows.end(),
                           [](const DebugWindow& w) { return !w.is_open; }),
            debug_windows.end());
    }

private:
    std::vector<DebugWindow> debug_windows;
    size_t next_window_id = 0; // Para dar nombres únicos a las nuevas ventanas
    bool global_pause_active = false; // ¡Nuevo! Estado de pausa global

    // Añade una nueva ventana de depuración
    void add_debug_window(const std::string& name) {
        // La nueva ventana hereda el estado de pausa global si está activa
        debug_windows.push_back({name, DebugEntityType::NONE, 0, true, global_pause_active, ""});
    }

    // Helper para cachear los datos de una ventana
    void cache_window_data(DebugWindow& debug_window, World& world) {
        std::stringstream ss;
        bool valid_index = false;

        switch (debug_window.selected_type) {
            case DebugEntityType::BODY:
                if (debug_window.entity_index < world.bodies.n_bodies) {
                    ss << get_generic_info(world.bodies, debug_window.entity_index, "BODY DATA");
                    valid_index = true;
                }
                break;
            case DebugEntityType::CONSTRAINT:
                if (debug_window.entity_index < world.constraints.n_constraints) {
                    ss << get_generic_info(world.constraints, debug_window.entity_index, "CONSTRAINT DATA");
                    valid_index = true;
                }
                break;
            case DebugEntityType::JOINT:
                if (debug_window.entity_index < world.joints.n_joints) {
                    ss << get_generic_info(world.joints, debug_window.entity_index, "JOINT DATA");
                    valid_index = true;
                }
                break;
            case DebugEntityType::NONE:
            default:
                break;
        }

        if (valid_index) {
            debug_window.cached_data = ss.str();
        } else {
            // Almacena un mensaje de error si el índice es inválido al pausar
            debug_window.cached_data = "Invalid Index or No Entity Selected!";
        }
    }


    // Función para renderizar una ventana de depuración individual
    void render_single_debug_window(DebugWindow& debug_window, World& world) {
        ImGui::Begin(debug_window.name.c_str(), &debug_window.is_open);

        // Botón de pausa individual para la ventana
        if (ImGui::Checkbox("Pause This Window", &debug_window.is_paused)) {
            if (debug_window.is_paused) {
                // Si la ventana se pausa, cachea los datos actuales
                cache_window_data(debug_window, world);
            }
            // Si se despausa, asegura que la pausa global también esté desactivada para esta ventana
            // (aunque la pausa global puede seguir activa para otras)
            if (!debug_window.is_paused && global_pause_active) {
                // Si despausas individualmente, la pausa global no se desactiva.
                // Podrías poner una lógica para desvincularla si lo prefieres.
            }
        }
        ImGui::SameLine();
        ImGui::Text("Status: %s", debug_window.is_paused ? "PAUSED" : "LIVE");

        ImGui::Separator();
        ImGui::Text("Monitored Data:");

        if (debug_window.is_paused) {
            // Si está pausado, muestra la información cacheada
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Displaying Cached Data:"); // Mensaje en verde
            ImGui::TextUnformatted(debug_window.cached_data.c_str());
        } else {
            // Si no está pausado, obtén y muestra la información en tiempo real
            // Input para el índice
            ImGui::InputInt("Entity Index", &debug_window.entity_index);
            if (debug_window.entity_index < 0) {
                debug_window.entity_index = 0;
            }

            const char* entity_types[] = {"None", "Body", "Constraint", "Joint"};
            int current_type_idx = static_cast<int>(debug_window.selected_type);
            if (ImGui::Combo("Entity Type", &current_type_idx, entity_types, IM_ARRAYSIZE(entity_types))) {
                debug_window.selected_type = static_cast<DebugEntityType>(current_type_idx);
            }

            ImGui::Separator();

            switch (debug_window.selected_type) {
                case DebugEntityType::BODY:
                    if (debug_window.entity_index < world.bodies.n_bodies) {
                        std::stringstream ss;
                        ss << get_generic_info(world.bodies, debug_window.entity_index, "BODY DATA");
                        ImGui::TextUnformatted(ss.str().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Invalid Body Index! Max: %ld", world.bodies.n_bodies - 1);
                    }
                    break;
                case DebugEntityType::CONSTRAINT:
                    if (debug_window.entity_index < world.constraints.n_constraints) {
                        std::stringstream ss;
                        ss << get_generic_info(world.constraints, debug_window.entity_index, "CONSTRAINT DATA");
                        ImGui::TextUnformatted(ss.str().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Invalid Constraint Index! Max: %ld", world.constraints.n_constraints - 1);
                    }
                    break;
                case DebugEntityType::JOINT:
                    if (debug_window.entity_index < world.joints.n_joints) {
                        std::stringstream ss;
                        ss << get_generic_info(world.joints, debug_window.entity_index, "JOINT DATA");
                        ImGui::TextUnformatted(ss.str().c_str());
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Invalid Joint Index! Max: %ld", world.joints.n_joints - 1);
                    }
                    break;
                case DebugEntityType::NONE:
                default:
                    ImGui::Text("Select an entity type to monitor.");
                    break;
            }
        }

        ImGui::End();
    }
};