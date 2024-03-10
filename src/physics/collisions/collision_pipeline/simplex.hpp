#pragma once
#include "physics/math/math.hpp"
#include "array"

/**
 * @brief Represents a simplex, a geometric object in a vector space defined by the convex hull of a set of points.
 *       This specific implementation is for 3d shapes so the simplex is a thetrahedron.
 * 
 * This class provides functionality to manipulate and access the vertices of the simplex.
 */
struct Simplex {
private:
    std::array<vec3, 4> m_points; /**< An array storing the vertices of the simplex. */
    int m_size; /**< The current number of vertices in the simplex. */

public:
    /**
     * @brief Default constructor for the Simplex class.
     * 
     * Initializes a Simplex object with zero vertices.
     */
    Simplex();

    /**
     * @brief Assigns a set of vertices to the Simplex using an initializer list.
     * 
     * @param list An initializer list containing vec3 vertices to assign to the Simplex.
     * @return A reference to the modified Simplex object.
     */
    Simplex& operator=(std::initializer_list<vec3> list);

    /**
     * @brief Adds a vertex to the front of the simplex.
     * 
     * @param point The vertex to add to the front of the simplex.
     */
    void push_front(vec3 point);

    /**
     * @brief Accesses the vertex at the specified index.
     * 
     * @param i The index of the vertex to access.
     * @return A reference to the vertex at the specified index.
     */
    vec3& operator[](int i);

    /**
     * @brief Returns the current number of vertices in the simplex.
     * 
     * @return The size of the simplex.
     */
    size_t size() const;

    /**
     * @brief Returns an iterator to the beginning of the simplex.
     * 
     * @return An iterator to the beginning of the simplex.
     */
    auto begin() const;

    /**
     * @brief Returns an iterator to the end of the simplex.
     * 
     * @return An iterator to the end of the simplex.
     */
    auto end() const;
};
