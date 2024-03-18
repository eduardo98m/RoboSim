#include "gjk.hpp"

bool same_direction(const vec3& direction, const vec3& ao)
{
	return ti::dot(direction, ao) > 0;
}
bool test_line(Simplex &points, vec3 &direction)
{
    vec3 a = points[0];
    vec3 b = points[1];

    vec3 ab = b - a;
    vec3 ao = -a;

    if (same_direction(ab, ao))
    {
        direction = ti::cross(ti::cross(ab, ao), ab);
    }

    else
    {
        points = {a};
        direction = ao;
    }

    return false;
}

bool test_triangle(Simplex &points, vec3 &direction)
{
    vec3 a = points[0];
    vec3 b = points[1];
    vec3 c = points[2];

    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 ao = -a;

    vec3 abc = ti::cross(ab, ac);

    if (same_direction(ti::cross(abc, ac), ao))
    {
        if (same_direction(ac, ao))
        {
            points = {a, c};
            direction = ti::cross(ti::cross(ac, ao), ac);
        }

        else
        {
            return test_line(points = {a, b}, direction);
        }
    }

    else
    {
        if (same_direction(ti::cross(ab, abc), ao))
        {
            return test_line(points = {a, b}, direction);
        }

        else
        {
            if (same_direction(abc, ao))
            {
                direction = abc;
            }

            else
            {
                points = {a, c, b};
                direction = -abc;
            }
        }
    }

    return false;
}

bool test_tetrahedron(Simplex& points, vec3& direction)
{
	vec3 a = points[0];
	vec3 b = points[1];
	vec3 c = points[2];
	vec3 d = points[3];

	vec3 ab = b - a;
	vec3 ac = c - a;
	vec3 ad = d - a;
	vec3 ao =   - a;
 
	vec3 abc = ti::cross(ab, ac);
	vec3 acd = ti::cross(ac, ad);
	vec3 adb = ti::cross(ad, ab);
 
	if (same_direction(abc, ao)) {
		return test_triangle(points = { a, b, c }, direction);
	}
		
	if (same_direction(acd, ao)) {
		return test_triangle(points = { a, c, d }, direction);
	}
 
	if (same_direction(adb, ao)) {
		return test_triangle(points = { a, d, b }, direction);
	}
 
	return true;
}

bool NextSimplex(Simplex& points, vec3& direction)
{
	switch (points.size()) {
		case 2: return test_line       (points, direction);
		case 3: return test_triangle   (points, direction);
		case 4: return test_tetrahedron(points, direction);
	}
 
	// never should be here
	return false;
}



bool gjk(const Collider& collider_1, const Collider& collider_2, Simplex* simplex)
{

    // Get initial support point in any direction
    vec3 support = support_function(collider_1, collider_2, vec3(1, 0, 0));

    // Simplex is an array of points, max count is 4
    Simplex points;
    points.push_front(support);

    // New direction is towards the origin
    vec3 direction = -support;


    // We limit the number of iterations  
    for (int i = 0; i < 100; i++)
    {

        support = support_function(collider_1, collider_2, direction);

        if (ti::dot(support, direction) <= 0)
        {
            return false; // no collision
        }

        points.push_front(support);

        if (NextSimplex(points, direction))
        {

            simplex = &points;
            return true;
        }
    }

    // The algorithm did not converge
    simplex = &points;
    return false;
}