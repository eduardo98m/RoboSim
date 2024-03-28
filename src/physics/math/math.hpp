#pragma once

#include "glm/glm.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "hpp/fcl/math/transform.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "raylib.h"

#ifdef EPSILON
#undef EPSILON
#endif
#define EPSILON 1e-7

#ifdef PI
#undef PI
#endif
#define PI 3.1415926

#ifdef SINGLE_PRECISION
typedef float scalar;
#elif DOUBLE_PRECISION
typedef double scalar;
#else
typedef double scalar;
#endif

typedef glm::vec<2, scalar> vec2;
std::string ToString(const vec2 &v, int precision = 4);
std::ostream &operator<<(std::ostream &os, const vec2 &v);

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

// We also define the pose struct which is somewhat useful as a return type for some functions:
struct pose
{
    vec3 position;
    quat orientation;
};

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

    using glm::acos;

    using glm::sqrt;

    using glm::max;
    using glm::min;

    using glm::abs;
    using glm::sign;

    /*l2 norm of the vector i.e: sqrt(x^2 + y^2 + z^2)*/
    scalar magnitude(vec3 v);

    quat quat_from_axis_angle(vec3 axis, scalar angle);

    /*
     *   Arctan 2 function. Calculates the arctan of the y/x fraction for the correspoding quadrant
     */
    scalar atan2(scalar y, scalar x);

    /*
     *
     */
    hpp::fcl::Transform3f get_eigen_transform(vec3 v, quat q);

    vec3 from_eigen(hpp::fcl::Vec3f v);

    mat3 mar3_from_eigen(const hpp::fcl::Matrix3f &mat);

    hpp::fcl::Vec3f to_eigen(vec3 v);

    Vector3 to_raylib(const vec3 &v);

    Quaternion to_raylib(const quat &q);

    quat from_eigen(hpp::fcl::Quaternion3f q);
};
