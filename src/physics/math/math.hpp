#pragma once

#include "glm/glm.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>

#define EPSILON 1e-7

#ifdef SINGLE_PRECISION
typedef float scalar;
#elif DOUBLE_PRECISION
typedef double scalar;
#else
typedef double scalar;
#endif

typedef glm::vec<3, scalar> vec3;
std::string ToString(const vec3 &v, int precision = 4);
std::ostream &operator<<(std::ostream &os, const vec3 &v);

typedef glm::qua<scalar> quat;
std::string ToString(const quat &v, int precision = 4);
std::ostream &operator<<(std::ostream &os, const quat &q);

typedef glm::mat<3, 3, scalar> mat3;
std::string ToString(const mat3 &m, int precision = 4);
std::ostream &operator<<(std::ostream &os, const mat3 &m);

typedef glm::mat<4, 4, scalar> mat4;
std::string ToString(const mat4 &m, int precision = 4);
std::ostream &operator<<(std::ostream &os, const mat4 &m);

namespace ti
{
    // Get the inverse function from the glm library
    using glm::inverse;
    // Get the cross product function from the glm library
    using glm::cross;
    // Get the dot product function from the glm library
    using glm::dot;
    // Get the normalize function from the glm library
    using glm::normalize;
    // Get the conjugate function from the glm library
    using glm::conjugate;
    // Get the transpose function from the glm library
    using glm::transpose;
    // Get the convert function from the glm library
    using glm::rotate;
    // Get the glm to convert a quaternion to a rotation matrix
    using glm::mat3_cast;
    // Get the glm to convert a rotation matrix to a quaternion
    using glm::quat_cast;

    using glm::clamp;
    
    using glm::cos;
    using glm::sin;

    /*l2 norm of the vector i.e: sqrt(x^2 + y^2 + z^2)*/
    scalar magnitude(vec3 v);

    quat quat_from_axis_angle(vec3 axis, scalar angle);

    /*
    *   Arctan 2 function. Calculates the arctan of the y/x fraction for the correspoding quadrant
    */
    scalar atan2(scalar y, scalar x);
};
