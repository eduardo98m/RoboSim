#pragma once
#include <memory>
#include <optional>

// TODO : Change this to another file
namespace rs
{
    struct Color
    {
        u_int8_t r = 255;
        u_int8_t g = 255;
        u_int8_t b = 255;
        u_int8_t a = 255;

        // Overloading the subscript operator []
        uint8_t &operator[](size_t index)
        {
            // Assuming index is within bounds (0 to 3)
            switch (index)
            {
            case 0:
                return r;
            case 1:
                return g;
            case 2:
                return b;
            case 3:
                return a;
            default:
                throw std::out_of_range("Index out of range");
            }
        }

        // Overloading const version of the subscript operator []
        const uint8_t &operator[](size_t index) const
        {
            // Assuming index is within bounds (0 to 3)
            switch (index)
            {
            case 0:
                return r;
            case 1:
                return g;
            case 2:
                return b;
            case 3:
                return a;
            default:
                throw std::out_of_range("Index out of range");
            }
        }
    };

};

// 
struct VisualShape{
    // Optional to the path of the .obj file
    std::optional<std::string> visual_object_path = std::nullopt;
    // Optional to the geometry (This is if the visual shape is not a mesh)
    std::optional<std::shared_ptr<hpp::fcl::CollisionGeometry>> geom = std::nullopt;
    // Position of the visual shape relative to the body center.
    vec3 pos = {0.0, 0.0, 0.0};
    // Orientation (rotation) of the visual shape relative to the body center
    quat rot = {1.0, 0.0, 0.0, 0.0};
    // Scale of the visual shape mesh
    vec3 scale = {1.0, 1.0, 1.0};
    // Color of the body
    rs::Color color = {.r = 255, .g = 95, .b = 31, .a = 255};
    // Body id to which the collider is attached to.
    size_t body_id;  
};


