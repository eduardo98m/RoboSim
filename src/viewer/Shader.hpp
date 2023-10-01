#pragma once

#include <string>
#include <glm/glm.hpp>

class Shader
{
public:
    unsigned int ID;
    /*
     * Constructor: reads and builds the shader
     * @param vertex_path: path to vertex shader
     * @param fragment_path: path to fragment shader
     * @param geometry_path: path to geometry shader (optional; default = nullptr)
     */
    Shader(const char *vertex_path, const char *fragment_path, const char *geometry_path = nullptr);
    /*
     * Activation function. (Activates the shader on openGL)
     */
    void use();

    // Utility uniform functions
    /* Sets a boolean uniform variable in the shader.
     * @param name: The name of the uniform variable in the shader.
     * @param value: The boolean value to set the uniform variable to.
     */
    void set_bool(const std::string &name, bool value) const;
    /* Sets an integer uniform variable in the shader.
     * @param name: The name of the uniform variable in the shader.
     * @param value: The integer value to set the uniform variable to.
     */
    void set_int(const std::string &name, int value) const;
    /* Sets a float uniform variable in the shader.
     * @param name: The name of the uniform variable in the shader.
     * @param value: The float value to set the uniform variable to.
     */
    void set_float(const std::string &name, float value) const;

    /* Sets a vec2 uniform variable in the shader using a glm::vec2.
     * @param name: The name of the uniform variable in the shader.
     * @param value: The glm::vec2 value to set the uniform variable to.
     */
    void set_vec2(const std::string &name, const glm::vec2 &value) const;
    /* Sets a vec2 uniform variable in the shader using two floats.
     * @param name: The name of the uniform variable in the shader.
     * @param x: The x component of the vec2.
     * @param y: The y component of the vec2.
     */
    void set_vec2(const std::string &name, float x, float y) const;

    /* Sets a vec3 uniform variable in the shader using a glm::vec3.
     * @param name: The name of the uniform variable in the shader.
     * @param value: The glm::vec3 value to set the uniform variable to.
     */
    void set_vec3(const std::string &name, const glm::vec3 &value) const;
    /* Sets a vec3 uniform variable in the shader using three floats.
     * @param name: The name of the uniform variable in the shader.
     * @param x: The x component of the vec3.
     * @param y: The y component of the vec3.
     * @param z: The z component of the vec3.
     */
    void set_vec3(const std::string &name, float x, float y, float z) const;
    /* Sets a vec3 uniform variable in the shader using three floats.
     * @param name: The name of the uniform variable in the shader.
     * @param x: The x component of the vec3.
     * @param y: The y component of the vec3.
     * @param z: The z component of the vec3.
     */
    void set_vec4(const std::string &name, const glm::vec4 &value) const;
    /* Sets a vec4 uniform variable in the shader using four floats.
     * @param name: The name of the uniform variable in the shader.
     * @param x: The x component of the vec4.
     * @param y: The y component of the vec4.
     * @param z: The z component of the vec4.
     * @param w: The w component of the vec4.
     */
    void set_vec4(const std::string &name, float x, float y, float z, float w);
    /* Sets a mat2 (2x2 matrix) uniform variable in the shader using a glm::mat2.
     * @param name: The name of the uniform variable in the shader.
     * @param mat: The glm::mat2 (2x2 matrix) to set the uniform variable to.
     */
    void set_mat2(const std::string &name, const glm::mat2 &mat) const;
    /* Sets a mat3 (3x3 matrix) uniform variable in the shader using a glm::mat3.
     * @param name: The name of the uniform variable in the shader.
     * @param mat: The glm::mat3 (3x3 matrix) to set the uniform variable to.
     */
    void set_mat3(const std::string &name, const glm::mat3 &mat) const;
    /* Sets a mat4 (4x4 matrix) uniform variable in the shader using a glm::mat4.
     * @param name: The name of the uniform variable in the shader.
     * @param mat: The glm::mat4 (4x4 matrix) to set the uniform variable to.
     */
    void set_mat4(const std::string &name, const glm::mat4 &mat) const;

private:
    /*
     * Utility function for checking shader compilation/linking errors.
     * @param shader: shader to check for errors
     * @param type: type ("VERTEX" or "FRAGMENT" or "FRAGMENT" or "PROGRAM") (TODO: CHECK IF THIS IS CORRECT)
     */
    void check_compile_errors(GLuint shader, std::string type);


};