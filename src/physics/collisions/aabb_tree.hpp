#pragma once

#include "broad_phase.hpp"
#include <vector>

class AABBTree {
public:
  AABBTree();

  void build(const std::vector<AABB>& aabb_list);
  
  void insert(const AABB& aabb, int body_id);
  
  std::vector<int> query(const AABB& aabb) const;

  void clear();
  
private:

  struct Node {
    AABB aabb;
    int body_id;
    Node* left; 
    Node* right;

    double volume; 
  };
  
  Node* root;

  void insert_recursive(Node* node, const AABB& aabb, int body_id);
  
  void query_recursive(const Node* node, const AABB& aabb, std::vector<int>& results) const;

  void clearRecursive(Node* node);

  double calculateAABBVolume(const AABB& aabb);
};

