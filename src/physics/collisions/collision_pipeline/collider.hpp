#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"
#include "collisions.hpp"
#include <memory>
enum class ColliderType
{
	NONE,		 ///< Empty shape type (Used fo bodies that do not have a collision shape )
	CONVEX_HULL, ///< Box shape.
	SPHERE,		 ///< Sphere shape.
	CAPSULE,	 ///< Capsule shape.
	// CYLINDER,   ///< Cylinder shape.
	PLANE,	  ///< Plane shape.
	HEIGHTMAP ///< Heightmap shape.
};

struct Collider
{
	ColliderType type;
	virtual vec3 find_furthest_point(const vec3 &direction) const = 0;
	Collider(ColliderType type) : type(type){};
};

struct SphereCollider : Collider
{
	vec3 center;
	scalar radius;
	SphereCollider(const vec3 &center, scalar radius) : center(center), radius(radius), Collider(ColliderType::SPHERE){};
	vec3 find_furthest_point(const vec3 &direction) const override
	{
		return this->center + this->radius * direction;
	};
};

struct PlaneCollider : Collider
{
	vec3 normal;
	scalar offset;

	PlaneCollider(const vec3 &normal, scalar offset) : normal(normal), offset(offset), Collider(ColliderType::PLANE){};
	vec3 find_furthest_point(const vec3 &direction) const override
	{
		// For a plane, the furthest point in a direction is either
		// along the normal direction or in the opposite direction of the normal
		// based on the relative orientation of the direction with the plane.
		if (ti::dot(direction, normal) > 0)
		{
			return normal * offset; // Along the normal
		}
		else
		{
			return -normal * offset; // Opposite to the normal
		}
	}
};

struct MeshCollider : Collider
{
	std::vector<vec3> vertices;
	std::vector<std::vector<size_t>> faces;
	std::vector<vec3> face_normals;


	MeshCollider(const std::vector<vec3> &vertices, const std::vector<std::vector<size_t>> &faces) : vertices(vertices), faces(faces), Collider(ColliderType::CONVEX_HULL){
		for (int i; i<faces.size();i++){
			face_normals.push_back(this->compute_face_normal(i));
		}
	};

	vec3 find_furthest_point(const vec3 &direction) const override
	{
		vec3 max_point;
		scalar max_distance = -INFINITY;

		for (const vec3 &vertex : vertices)
		{
			scalar distance = ti::dot(vertex, direction);
			if (distance > max_distance)
			{
				max_distance = distance;
				max_point = vertex;
			}
		}

		return max_point;
	}

	size_t support_point_get_index(vec3 direction)
	{
		size_t selected_index = 0;
		scalar max_dot = -INFINITY;
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			scalar dot = ti::dot(this->vertices[i], direction);
			if (dot > max_dot)
			{
				selected_index = i;
				max_dot = dot;
			}
		}

		return selected_index;
	}

	vec3 compute_face_normal(size_t face_index){
		vec3 a = this->vertices[this->faces[face_index][0]];
		vec3 b = this->vertices[this->faces[face_index][1]];
		vec3 c = this->vertices[this->faces[face_index][2]];
		return ti::normalize(ti::cross(b - a, c-b));
	}

	vec3 compute_face_center(size_t face_index){

		vec3 center = {0.0, 0.0, 0.0};
		int n_vertices = 0;
		for (size_t idx: this->faces[face_index]){
			center = this->vertices[idx];
			n_vertices++;
		}

		

		return center / (scalar)n_vertices;
	}
};

std::unique_ptr<Collider> cast_shape_to_collider(const ShapeInfo &shape, const vec3 &position, const quat &orientation);