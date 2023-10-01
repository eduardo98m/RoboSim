#pragma once

#include "Geometry.hpp"
#include "Shader.hpp"

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <vector>
#include <set>
#include <map>

const int MAX_BONE_INFLUENCE = 4;

struct Vertex
{
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
    glm::vec3 Tangent;
    glm::vec3 Bitangent;
    int BoneIds[MAX_BONE_INFLUENCE] = {0};
    float BoneWeights[MAX_BONE_INFLUENCE] = {0.0f};
};

enum TextureType
{
    TexAlbedo,
    TexNormal,
    TexMetalness,
    TexRoughness,
    TexEmission,
    TexAmbientOcclusion,
    TexSpecular,
    TexLast
};

struct Texture
{
public:
    unsigned int id;
    std::set<TextureType> types;
    std::string path;
    int num_channels;

    Texture(const std::string &base_name);

    static std::string get_short_name_of_texture_type(TextureType texture_type);

    static std::string get_long_name_of_texture_type(TextureType texture_type);

    std::string get_name();

private:
    std::string base_name;
};

enum FileFormat
{
    Default,
    glTF,
    FBX
};

struct Material
{
    std::string name;
    std::map<TextureType, Texture *> textures;
    FileFormat format;

    Material(const std::string &name)
    {
        this->name = name;
    }
};

class Mesh
{
public:
    // Name of the mesh
    std::string name;
    // Vertices
    std::vector<Vertex> vertices;
    // Indices
    std::vector<unsigned int> indices;
    // Material 
    Material *material;
    unsigned int VAO;

    // constructor
    Mesh(const std::string &name, std::vector<Vertex> &vertices, std::vector<unsigned int> &indices, Material *material);

    // render the mesh
    void draw(Shader *shader, Material *draw_material, bool is_selected, bool disable_depth_test, bool render_only_ambient, bool render_one_color);

    bool intersected_ray(const glm::vec3 &orig, const glm::vec3 &dir, float &t);

private:
    // render data
    unsigned int VBO, EBO;

    // @brief all the buffer objects/arrays
    // 
    void setup_mesh();
};