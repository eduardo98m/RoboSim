#pragma once

#include "glm/glm.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#define EPSILON 1e-7

#ifdef SINGLE_PRECISION
typedef float scalar;
#elif DOUBLE_PRECISION
typedef double scalar;
#else
typedef double scalar;
#endif



// Create a 3d vector type with scalar components that we will use as our base vector type
typedef glm::vec<3, scalar> vec3;
// Create a quaternion type with scalar components that we will use as our base quaternion type
typedef glm::qua<scalar> quat;
// Create a 3x3 matrix type with scalar components that we will use as our base matrix type
// It is used for the inertia tensor, and the rotation matrix
typedef glm::mat<3, 3, scalar> mat3;
// Create a 4x4 matrix type with scalar components that we will use as our base matrix type
// It is used for the transformation matrix
typedef glm::mat<4, 4, scalar> mat4;

namespace ti{
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
    // Get the magnitude function from the glm library
    using glm::length;
    // Get the transpose function from the glm library
    using glm::transpose;
    // Get the convert function from the glm library
    using glm::rotate;
    // Get the glm to convert a quaternion to a rotation matrix
    using glm::mat3_cast;
};

