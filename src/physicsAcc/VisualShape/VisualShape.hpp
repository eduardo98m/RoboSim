#pragma once
#include <raylib.h>

// Internal
#include "physics/math/math.hpp"


enum VisualShapeType{
    BOX,
    SPHERE,
    CYLINDER,
    HEIGHTMAP,
    PLANE,

};

struct VisualShape
{
    size_t id = -1; // Id of the visual shape in the visualizer
    vec3 position = {0.0, 0.0, 0.0}; 
    quat orientation = {0.0, 0.0, 0.0, 1.0};
    VisualShapeType type = VisualShapeType::SPHERE; // (Helps search for the visual shape in the visualizer)
};
