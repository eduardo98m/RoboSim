#include "math.hpp"

namespace ti
{
    scalar magnitude(vec3 v)
    {
        return glm::l2Norm(v);
    }
};

std::string ToString(const vec3 &v, int precision)
{
    std::stringstream ss;
    ss << "[ ";
    ss << std::fixed << std::setprecision(precision) << v.x << ", ";
    ss << std::fixed << std::setprecision(precision) << v.y << ", ";
    ss << std::fixed << std::setprecision(precision) << v.z;
    ss << " ]";
    return ss.str();
}

std::ostream &operator<<(std::ostream &os, const vec3 &v)
{
    os << ToString(v, 4);
    return os;
}

std::string ToString(const quat &q, int precision)
{
    std::stringstream ss;
    ss << "[ ";
    ss << std::fixed << std::setprecision(precision) << q.x << ", ";
    ss << std::fixed << std::setprecision(precision) << q.y << ", ";
    ss << std::fixed << std::setprecision(precision) << q.z << ", ";
    ss << std::fixed << std::setprecision(precision) << q.w;
    ss << " ]";
    return ss.str();
}

std::ostream &operator<<(std::ostream &os, const quat &q)
{
    os << ToString(q, 4);
    return os;
}

std::string ToString(const mat3 &m, int precision)
{
    std::stringstream ss;
    ss << "[\n";
    for (int i = 0; i < 3; i++)
    {
        ss << "  [ ";
        for (int j = 0; j < 3; j++)
        {
            ss << std::fixed << std::setprecision(precision) << m[i][j];
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

std::ostream &operator<<(std::ostream &os, const mat3 &m)
{
    os << ToString(m, 4);
    return os;
}

std::string ToString(const mat4 &m, int precision)
{
    std::stringstream ss;
    ss << "[\n";
    for (int i = 0; i < 4; i++)
    {
        ss << "  [ ";
        for (int j = 0; j < 4; j++)
        {
            ss << std::fixed << std::setprecision(precision) << m[i][j];
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

std::ostream &operator<<(std::ostream &os, const mat4 &m)
{
    os << ToString(m, 4);
    return os;
}
