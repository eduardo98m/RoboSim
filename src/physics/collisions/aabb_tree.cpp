#include "aabb_tree.hpp"

AABBTree::AABBTree() : root(nullptr) {}

void AABBTree::build(const std::vector<AABB> &aabb_list)
{
    if (aabb_list.empty())
    {
        return;
    }

    // Clear the existing tree
    this->clear();
    for (int i = 0; i < aabb_list.size(); i++)
    {
        this->insert(aabb_list[i], i);
    }
}

void AABBTree::insert(const AABB &aabb, size_t collider_id)
{

    if (this->root == nullptr)
    {
        this->root = new Node{
            .aabb = aabb,
            .collider_id = collider_id};
        
        nodes.push_back(this->root);
    }
    else
    {   
        this->insert_recursive(this->root, aabb, collider_id);
    }
}

void AABBTree::insert_recursive(Node *node, const AABB &aabb, size_t collider_id)
{

    // Create the combined node

    if (node->is_leaf())
    {   
        // Check collisions with the node
        this->query_recursive(node, aabb, collider_id);

        Node *child_left = new Node{
            .aabb = node->aabb,
            .collider_id = node->collider_id,
            .parent = node,
        };

        Node* child_right = new Node{
            .aabb = aabb,
            .collider_id = collider_id,
            .parent = node
        };

        // We morph the leaf node into a regular node
        node->aabb = combine_aabbs(node->aabb, aabb);
        node->collider_id = NULL_COLLIDER;
        node->left = child_left;
        node->right = child_right;

        nodes.push_back(child_left);
        nodes.push_back(child_right);

    }
    else
    {
        // Use the heuristic to select where to add the new node (left or right)
        // The heuristic that we will use has the objective of maintaining a balanced tree using as cost the volume of the aabbs
        node->aabb = combine_aabbs(node->aabb, aabb);

        scalar left_volume = calculate_aabb_volume(combine_aabbs(node->left->aabb, aabb));
        scalar right_volume = calculate_aabb_volume(combine_aabbs(node->right->aabb, aabb));

        if (left_volume < right_volume)
        {
            this->insert_recursive(node->left, aabb, collider_id);
            this->query_recursive(node->right, aabb, collider_id);
        }
        else
        {
            this->insert_recursive(node->right, aabb, collider_id);
            this->query_recursive(node->left, aabb, collider_id);
        }
    }
}


void AABBTree::query_recursive(Node *node,
                               const AABB &aabb,
                               size_t collider_id)
{

    // This should be an assertion
    if (node == nullptr)
    {
        std::cerr << "AABB Tree has error in his structure, non leaf node has only one child\n";
        return;
    }

    if (!check_broad_phase_collision(node->aabb, aabb))
    {
        return;
    }

    if (node->is_leaf())
    {
        this->collisions.push_back(std::make_pair(node->collider_id, collider_id));
    }
    else
    {
        this->query_recursive(node->left, aabb, collider_id);
        this->query_recursive(node->right, aabb, collider_id);
    }
}


void AABBTree::query_plane_recursive(Node *node,
                               const hpp::fcl::Plane &plane,
                               size_t collider_id)
{

    // This should be an assertion
    if (node == nullptr)
    {
        std::cerr << "AABB Tree has error in his structure, non leaf node has only one child\n";
        return;
    }

    if (!check_broad_phase_collision(plane, node->aabb))
    {
        return;
    }

    if (node->is_leaf())
    {
        this->collisions.push_back(std::make_pair(node->collider_id, collider_id));
    }
    else
    {
        this->query_plane_recursive(node->left, plane, collider_id);
        this->query_plane_recursive(node->right, plane, collider_id);
    }
}


void AABBTree::query_plane(const hpp::fcl::Plane &plane, size_t collider_id){
    // This should be an assertion
    this->query_plane_recursive(this->root, plane, collider_id);
}

void AABBTree::clear()
{
    this->nodes.clear();
    if (this->root == nullptr)
    {
        return;
    }

    clear_recursive(this->root);
    this->root = nullptr;
}

void AABBTree::clear_recursive(Node *node)
{

    if (node->left != nullptr)
    {
        clear_recursive(node->left);
    }

    if (node->right != nullptr)
    {
        clear_recursive(node->right);
    }

    delete node;
}


void AABBTree::printTree(Node* node, int depth) const {
    if (node == nullptr) {
        return;
    }

    // Indentation for visual clarity
    for (int i = 0; i < depth; ++i) {
        std::cout << "  ";
    }

    // Print node information
    std::cout << "Node: " << node << std::endl;
    //std::cout << "  AABB: " << node->collider_id << std::endl; // Assuming AABB has an appropriate output operator
    std::cout << "  Is Leaf: " << (node->is_leaf() ? "true" : "false") << std::endl;
    if (node->is_leaf()) {
        std::cout << "  Collider ID: " << node->collider_id << std::endl;
    }

    // Recursively print children
    printTree(node->left, depth + 1);
    printTree(node->right, depth + 1);
}

void AABBTree::print() const {
    this->printTree(this->root);
};

scalar calculate_aabb_volume(const AABB &aabb)
{

    scalar dx = aabb.max.x - aabb.min.x;
    scalar dy = aabb.max.y - aabb.min.y;
    scalar dz = aabb.max.z - aabb.min.z;

    return dx * dy * dz;
}

AABB combine_aabbs(const AABB &A, const AABB &B)
{
    return AABB{
        .min = {ti::min(A.min.x, B.min.x), ti::min(A.min.y, B.min.y), ti::min(A.min.z, B.min.z)},
        .max = {ti::max(A.max.x, B.max.x), ti::max(A.max.y, B.max.y), ti::max(A.max.z, B.max.z)},
    };
}

bool Node::is_leaf()
{
    return (this->left == nullptr && this->right == nullptr);
}