#pragma once

#include "glm/glm.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
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

// // Create a 3d vector type with scalar components that we will use as our base vector type
// typedef glm::vec<3, scalar> vec3;
// // Create a quaternion type with scalar components that we will use as our base quaternion type
// typedef glm::qua<scalar> quat;
// // Create a 3x3 matrix type with scalar components that we will use as our base matrix type
// // It is used for the inertia tensor, and the rotation matrix
// typedef glm::mat<3, 3, scalar> mat3;
// // Create a 4x4 matrix type with scalar components that we will use as our base matrix type
// // It is used for the transformation matrix
// typedef glm::mat<4, 4, scalar> mat4;

// Extend glm::vec3
struct vec3 : glm::vec<3, scalar>
{
    using glm::vec<3, scalar>::vec;
    void setZero()
    {
        *this = glm::vec<3, scalar>(0.0);
    }

    void setConstant(scalar x)
    {
        *this = glm::vec<3, scalar>(x);
    }

    std::string ToString(int precision = 4) const
    {
        std::stringstream ss;
        ss << "[ ";
        ss << std::fixed << std::setprecision(precision) << this->x << ", ";
        ss << std::fixed << std::setprecision(precision) << this->y << ", ";
        ss << std::fixed << std::setprecision(precision) << this->z;
        ss << " ]";
        return ss.str();
    }
};

// Extend glm::quat
struct quat : glm::qua<scalar>
{
    using glm::qua<scalar>::qua;
    void setZero()
    {
        *this = glm::qua<scalar>(1.0, 0.0, 0.0, 0.0);
    }

    void setIdentity()
    {
        *this = glm::qua<scalar>(1.0, 0.0, 0.0, 0.0);
    }

    std::string ToString(int precision = 4) const
    {
        std::stringstream ss;
        ss << "[ ";
        ss << std::fixed << std::setprecision(precision) << this->x << ", ";
        ss << std::fixed << std::setprecision(precision) << this->y << ", ";
        ss << std::fixed << std::setprecision(precision) << this->z << ", ";
        ss << std::fixed << std::setprecision(precision) << this->w;
        ss << " ]";
        return ss.str();
    }
};

// Extend glm::mat3
struct mat3 : glm::mat<3, 3, scalar>
{
    using glm::mat<3, 3, scalar>::mat;
    void setZero()
    {
        *this = glm::mat<3, 3, scalar>(0.0);
    }

    void setIdentity()
    {
        *this = glm::identity<glm::mat<3, 3, scalar>>();
    }

    // Method to convert the matrix to a string with a specified precision (default: 4)
    std::string ToString(int precision = 4) const
    {
        std::stringstream ss;
        ss << "[\n";
        for (int i = 0; i < 3; i++)
        {
            ss << "  [ ";
            for (int j = 0; j < 3; j++)
            {
                ss << std::fixed << std::setprecision(precision) << (*this)[i][j];
                if (j < 2)
                {
                    ss << ", ";
                }
            }
            ss << " ]";
            if (i < 2)
            {
                ss << ",\n";
            }
            else
            {
                ss << "\n";
            }
        }
        ss << "]";
        return ss.str();
    }
};

// Extend glm::mat4
struct mat4 : glm::mat<4, 4, scalar>
{
    using glm::mat<4, 4, scalar>::mat;
    void setZero()
    {
        *this = glm::mat<4, 4, scalar>(0.0);
    }

    void setIdentity()
    {
        *this = glm::identity<glm::mat<4, 4, scalar>>();
    }

    // Method to convert the matrix to a string with a specified precision (default: 4)
    std::string ToString(int precision = 4) const
    {
        std::stringstream ss;
        ss << "[\n";
        for (int i = 0; i < 4; i++)
        {
            ss << "  [ ";
            for (int j = 0; j < 4; j++)
            {
                ss << std::fixed << std::setprecision(precision) << (*this)[i][j];
                if (j < 3)
                {
                    ss << ", ";
                }
            }
            ss << " ]";
            if (i < 3)
            {
                ss << ",\n";
            }
            else
            {
                ss << "\n";
            }
        }
        ss << "]";
        return ss.str();
    }
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

    /*l2 norm of the vector i.e: sqrt(x^2 + y^2 + z^2)*/
    scalar magnitude(vec3 v)
    {
        return glm::l2Norm(v);
    }
};
