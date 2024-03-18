#include "aabb_tree.hpp"

AABBTree::AABBTree() : root(nullptr) {}

void AABBTree::build(const std::vector<AABB>& aabb_list) {
    if (aabb_list.empty()) {
        return;
    }

    // Clear the existing tree
    this->clear();
    for (int i = 0 ; i < aabb_list.size(); i++){
        this->insert(aabb_list[i], i);
    }
}

void AABBTree::insert(const AABB& aabb, int body_id) {
  if (root == nullptr) {
    root = new Node({aabb, -1, nullptr, nullptr, .volume  = this->calculateAABBVolume(aabb)});
  } else {
    insert_recursive(root, aabb, body_id); 
  }
}

void AABBTree::insert_recursive(Node* node, const AABB& aabb, int body_id) {
  
  double volume = this->calculateAABBVolume(aabb);
  // Insert leaf 
  if (node->body_id == -1) {
    node->aabb = aabb;
    node->body_id = body_id;
    node->volume = this->calculateAABBVolume(aabb); 
    return;
  }

  // Insert on left or right
  if (aabb.min.x < node->aabb.min.x) {
    if (node->left == nullptr) {
      node->left = new Node({aabb, -1, nullptr, nullptr,  .volume = volume}); 
    }
    insert_recursive(node->left, aabb, body_id);
  } else {
    if (node->right == nullptr) {
      node->right = new Node({aabb, -1, nullptr, nullptr, .volume = volume});
    }
    insert_recursive(node->right, aabb, body_id);
  }

  // Update node AABB 
  node->aabb = merge_aabb(node->aabb, aabb);
  node->volume = this->calculateAABBVolume(node->aabb);
}


std::vector<int> AABBTree::query(const AABB& aabb) const {
  std::vector<int> results;
  if (root != nullptr) {
    query_recursive(root, aabb, results);
  }
  return results;
}

void AABBTree::query_recursive(const Node* node, 
                               const AABB& aabb, 
                               std::vector<int>& results) const {
  
  if (!check_broad_phase_collision(node->aabb, aabb)) {
    return;
  }

  if (node->body_id != -1) {
    results.push_back(node->body_id);
  }

  if (node->left != nullptr) {
    query_recursive(node->left, aabb, results);
  }

  if (node->right != nullptr) {
    query_recursive(node->right, aabb, results);
  }

}

void AABBTree::clear() {

  if (root == nullptr) {
    return; 
  }

  clearRecursive(root);
  root = nullptr;

}

void AABBTree::clearRecursive(Node* node) {
  
  if (node->left != nullptr) {
    clearRecursive(node->left);  
  }

  if (node->right != nullptr) {
    clearRecursive(node->right);
  }

  delete node;
}

double AABBTree::calculateAABBVolume(const AABB& aabb) {

  double dx = aabb.max.x - aabb.min.x;
  double dy = aabb.max.y - aabb.min.y;
  double dz = aabb.max.z - aabb.min.z;

  return dx * dy * dz;
}