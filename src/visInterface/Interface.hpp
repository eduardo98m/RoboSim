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

struct VisualizationSettings{
    bool show_revolute_joints = false;
    bool show_bounding_boxes = false;
    bool show_contact_points = true;
    int size = 50;
    std::vector<std::pair<int, bool>> enabled_rev_joints;
};

class Interface
{
private:
    std::vector<std::pair<int, int>> body_to_visual_shape;

    std::shared_ptr<robosim::World> world_;
    std::shared_ptr<Visualizer> visualizer_;
    int plane_idx;

    VisualizationSettings settings;
public:
    Interface(std::shared_ptr<robosim::World> world, std::shared_ptr<Visualizer> visualizer);

    void update(void);
    
};



