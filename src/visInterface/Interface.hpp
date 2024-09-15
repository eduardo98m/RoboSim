#pragma once
#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"
#include "Visualizer.hpp"
#include "physics/World.hpp"
#include "broad_phase.hpp"

// External libs
#include <raylib.h>
#include <raymath.h>
#include <memory>
#include "imgui.h"

//
#include <iomanip>
#include <sstream>


#define VISUAL_SHAPES_GROUP 1

#define COLLISION_SHAPES_GROUP 2

struct VisualizationSettings{

    bool show_revolute_joints = false;
    bool show_bounding_boxes = false;
    bool show_contact_points = false;
    bool render_visual_objects = true;
    bool render_collision_shapes = false;

    bool show_aabb_tree = false;

    bool toggle_visual_objects = false;
    bool toggle_collision_shapes = false;

    int size = 50;
    std::vector<std::pair<int, bool>> enabled_rev_joints;
};

class Interface
{
private:
    std::vector<std::pair<int, int>> body_to_visual_shape;
    std::vector<std::pair<int, int>> collider_to_collision_shape;

    std::shared_ptr<robosim::World> world_;
    std::shared_ptr<Visualizer> visualizer_;
    int plane_idx;

    VisualizationSettings settings;
public:
    Interface(std::shared_ptr<robosim::World> world, std::shared_ptr<Visualizer> visualizer);

    void update(void);

    void add_collision_object(int body_id, int group_id,  Color* color, bool visual_shape = false);
    
    void add_visual_object(int body_id, int group_id);
};



