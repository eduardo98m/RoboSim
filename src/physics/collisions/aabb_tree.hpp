#pragma once

#include "broad_phase.hpp"
#include <vector>

#define NULL_COLLIDER 0xffffff

struct Node
{
    AABB aabb;
    size_t collider_id = NULL_COLLIDER;

    // Indexeds of adyancent nodes
    Node *left = nullptr;
    Node *right = nullptr;
    Node *parent = nullptr;

    // // Depth of the node in the Tree
    // size_t depth;

    // Id of the collider (Only for leaf nodes)
    bool is_leaf();
};

class AABBTree
{
public:
    AABBTree();

    void build(const std::vector<AABB> &aabb_list);

    void insert(const AABB &aabb, size_t collider_id);

    std::vector<int> query(const AABB &aabb) const;

    void clear();

    // List of colliders_ids of which its aabb have collided
    std::vector<std::pair<size_t, size_t>> collisions;

    std::vector<Node *> nodes;
    void print() const;

    void query_plane(const hpp::fcl::Plane &plane, size_t collider_id);

private:
    Node *root;

    size_t node_count;

    void insert_recursive(Node *node, const AABB &aabb, size_t collider_id);

    void query_recursive(Node *node,
                         const AABB &aabb,
                         size_t collider_id);

    void query_plane_recursive(Node *node,
                               const hpp::fcl::Plane &aabb,
                               size_t collider_id);

    void clear_recursive(Node *node);

    void printTree(Node *node, int depth = 0) const;
};

AABB combine_aabbs(const AABB &A, const AABB &B);

scalar calculate_aabb_volume(const AABB &aabb);
