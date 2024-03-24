#include "math.hpp"

namespace ti
{
    scalar magnitude(vec3 v)
    {
        return glm::l2Norm(v);
    }

    quat quat_from_axis_angle(vec3 axis, scalar angle)
    {
        return glm::angleAxis(angle, axis);
    }

    scalar atan2(scalar y, scalar x)
    {
        return std::atan2(y, x);
    }

    hpp::fcl::Transform3f get_eigen_transform(vec3 v, quat q)
    {
        hpp::fcl::Transform3f T;
        T.setQuatRotation(hpp::fcl::Quaternion3f(q.w, q.x, q.y, q.z));
        T.setTranslation(hpp::fcl::Vec3f(v.x, v.y, v.z));
        return T;
    }

    vec3 from_eigen(hpp::fcl::Vec3f v)
    {

        return (vec3){v[0], v[1], v[2]};
    }

    mat3 mar3_from_eigen(const hpp::fcl::Matrix3f &mat){
        return (mat3){
            mat(0, 0), mat(0, 1), mat(0, 2),
            mat(0, 1), mat(1, 1), mat(1, 2),
            mat(0, 2), mat(1, 2), mat(2, 2)
        };
    }

    quat from_eigen(hpp::fcl::Quaternion3f q)
    {
        return (quat){q.x(), q.y(), q.z(), q.w()};
    }


    hpp::fcl::Vec3f to_eigen(vec3 v)
    {

        return (hpp::fcl::Vec3f){v[0], v[1], v[2]};
    }

    Vector3 to_raylib(const vec3 &v)
    {
        return {(float)v.x, (float)v.y, (float)v.z};
    }

    Quaternion to_raylib(const quat &q)
    {
        return {(float)q.x, (float)q.y, (float)q.z, (float)q.w};
    }

    


};

std::string ToString(const vec2 &v, int precision)
{
    std::stringstream ss;
    ss << "[ ";
    ss << std::fixed << std::setprecision(precision) << v.x << ", ";
    ss << std::fixed << std::setprecision(precision) << v.y;
    ss << " ]";
    return ss.str();
}

std::ostream &operator<<(std::ostream &os, const vec2 &v)
{
    os << ToString(v, 4);
    return os;
}

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
