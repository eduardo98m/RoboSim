#include "physicsAcc/Colliders/Collider.hpp"

bool set_pose(ColliderCollection &cc, size_t i, const vec3 &position,
              const quat &orientation) {
  if (i > cc.n_colliders) {
    return false;
  }
  cc.collider[i]->setTransform(ti::get_eigen_transform(position, orientation));
  // Update the collision manager
  cc.collision_manager.update(cc.collider[i].get());
  return true;
}

void update_collider_poses(ColliderCollection &cc, const BodyCollection &bc,
                           scalar timestep) {
  for (size_t i = 0; i < cc.n_colliders; i++) {
    ColliderUserData *data =
        static_cast<ColliderUserData *>(cc.collider[i]->getUserData());
    size_t body = data->body_id;
    // Skip static bodies
    if (bc.type[body] == BodyType::STATIC)
      continue;
    vec3 pos =
        bc.position[body] + ti::rotate(bc.orientation[body], data->position);
    quat rot = bc.orientation[body] * data->orientation;
    cc.collider[i]->setTransform(ti::get_eigen_transform(pos, rot));
    // Update the collision manager (AABB tree)
    scalar aabb_radius = cc.collider[i]->collisionGeometry()->aabb_radius;
    // We expand the aabb and update the collision manager
    scalar expansion_factor = 2.0;
    cc.collider[i]->collisionGeometry()->aabb_radius =
        aabb_radius +
        expansion_factor * timestep * ti::magnitude(bc.linear_velocity[i]);
    cc.collision_manager.update(cc.collider[i].get());
    // We set the aabb_radius to the original one again
    cc.collider[i]->collisionGeometry()->aabb_radius = aabb_radius;
  }
}

BroadPhaseResult broad_phase_collision_detection(ColliderCollection &cc) {

  BroadPhaseCallBack callback;

  // Update the collider info on the tree (We can make this more efficient only
  // updating the bodies that move)
  for (size_t i = 0; i < cc.n_colliders; i++) {
    cc.collision_manager.update(cc.collider[i].get());
  }
  // Perform broadphase collision detection
  cc.collision_manager.collide(&callback);

  return callback.result;
}

void solve_contact(ContactCollection &contact_collection, size_t contact_id,
                   ColliderCollection &cc, size_t collider_a, size_t collider_b,
                   BodyCollection &bc, size_t body_a, size_t body_b,
                   scalar inverse_time_step) {

  apply_constraint_position_level(contact_collection, contact_id, bc,
                                  inverse_time_step);

  // Update the colliders position and orientation
  ColliderUserData *data_a =
      static_cast<ColliderUserData *>(cc.collider[collider_a]->getUserData());
  vec3 pos_a = bc.position[body_a] +
               ti::rotate(bc.orientation[body_a], data_a->position);
  quat rot_a = bc.orientation[body_a] * data_a->orientation;
  cc.collider[collider_a]->setTransform(ti::get_eigen_transform(pos_a, rot_a));

  ColliderUserData *data_b =
      static_cast<ColliderUserData *>(cc.collider[collider_b]->getUserData());
  vec3 pos_b = bc.position[body_b] +
               ti::rotate(bc.orientation[body_b], data_b->position);
  quat rot_b = bc.orientation[body_b] * data_b->orientation;
  cc.collider[collider_b]->setTransform(ti::get_eigen_transform(pos_b, rot_b));
}

ContactCollection
narrow_phase_collision(ColliderCollection &cc, BodyCollection &bc,
                       const BroadPhaseResult &broad_phase_pairs,
                       scalar inverse_time_step) {

  ContactCollection contact_collection;

  for (size_t i = 0; i < broad_phase_pairs.n_possible_collisions; i++) {
    hpp::fcl::CollisionResult col_res;
    hpp::fcl::CollisionRequest col_req =
        hpp::fcl::CollisionRequest(hpp::fcl::CollisionRequestFlag::CONTACT, 16);

    size_t col_a = broad_phase_pairs.collider_1[i];
    size_t col_b = broad_phase_pairs.collider_2[i];

    hpp::fcl::collide(cc.collider[col_a].get(), cc.collider[col_b].get(),
                      col_req, col_res);

    for (const auto contact : col_res.getContacts()) {

      ColliderUserData *data_a =
          static_cast<ColliderUserData *>(cc.collider[col_a]->getUserData());
      ColliderUserData *data_b =
          static_cast<ColliderUserData *>(cc.collider[col_b]->getUserData());
      size_t cotact_id = contact_collection.n_contacts;
      contact_collection.n_contacts++;
      contact_collection.body_1.push_back(data_a->body_id);
      contact_collection.body_2.push_back(data_b->body_id);
      contact_collection.dynamic_friction.push_back(
          data_a->dynamic_friction * 0.5 + data_b->dynamic_friction * 0.5);
      contact_collection.static_friction.push_back(
          data_a->static_friction * 0.5 + data_b->static_friction * 0.5);
      contact_collection.restitution.push_back(
          ti::min(data_a->restitution, data_b->restitution));
      contact_collection.normal.push_back(ti::from_eigen(contact.normal));
      contact_collection.p_1.push_back(ti::from_eigen(
          contact.pos + contact.normal * contact.penetration_depth * 0.5));
      contact_collection.p_2.push_back(ti::from_eigen(
          contact.pos - contact.normal * contact.penetration_depth * 0.5));
      // We fill the rest of vectors with placeholder values
      contact_collection.collision.push_back(false);
      contact_collection.normal_force.push_back(vec3(0.0));
      contact_collection.tangencial_force.push_back(vec3(0.0));
      contact_collection.normal_constraint_lagrange_multiplier.push_back(0.0);
      contact_collection.tangencial_constraint_lagrange_multiplier.push_back(
          0.0);
      contact_collection.relative_velocity.push_back(0.0);
      // We solve the contact when as soon it is detected
      solve_contact(contact_collection, cotact_id, cc, col_a, col_b, bc,
                    data_a->body_id, data_b->body_id, inverse_time_step);
    }
  }

  return contact_collection;
}

/**
 * @brief Gets independent collision groups for a given set of detected
 * collisions [Check the greedy coloring part]
 * -> Use greedy graph coloring
 */
std::vector<std::vector<size_t>>
get_independent_collision_groups(const BroadPhaseResult &result,
                                 const ColliderCollection &cc) {
  // Map of bodies with bodies (i.e the graph)
  std::unordered_map<size_t, std::unordered_set<size_t>> graph;
  // Map : body_to_to the collision
  std::unordered_map<size_t, std::vector<size_t>> body_to_collision;

  std::vector<std::pair<size_t, size_t>> collision_bodies(
      result.n_possible_collisions);

  for (size_t i = 0; i < result.n_possible_collisions; ++i) {
    size_t collider_a = result.collider_1[i];
    size_t collider_b = result.collider_2[i];

    auto *data_a =
        static_cast<ColliderUserData *>(cc.collider[collider_a]->getUserData());
    auto *data_b =
        static_cast<ColliderUserData *>(cc.collider[collider_b]->getUserData());

    size_t body_a = data_a->body_id;
    size_t body_b = data_b->body_id;

    collision_bodies[i] = {body_a, body_b};

    // Add the information to the graph
    graph[body_a].insert(body_b);
    graph[body_b].insert(body_a);

    body_to_collision[body_a].push_back(i);
    body_to_collision[body_b].push_back(i);
  }
  // Graph coloring
  // Map: body_id -> assigned color
  std::unordered_map<size_t, size_t> body_colors;

  // Create an order of nodes (could be arbitrary or by descending degree for
  // better results)
  std::vector<size_t> nodes;
  for (const auto &[node, _] : graph)
    nodes.push_back(node);

  // Optional: sort nodes by descending degree for better greedy coloring
  std::sort(nodes.begin(), nodes.end(), [&](size_t a, size_t b) {
    return graph[a].size() > graph[b].size();
  });

  // Greedy coloring
  for (size_t node : nodes) {
    std::unordered_set<size_t> neighbor_colors;
    for (size_t neighbor : graph[node]) {
      if (body_colors.find(neighbor) != body_colors.end()) {
        neighbor_colors.insert(body_colors[neighbor]);
      }
    }

    // Find the first available color (same as first_available function)
    size_t color = 0;
    while (neighbor_colors.count(color))
      ++color;

    body_colors[node] = color;
  }

  // Group collisions by color
  std::unordered_map<size_t, std::vector<size_t>> color_to_collisions;
  for (const auto &[body_id, color] : body_colors) {
    for (size_t collision_idx : body_to_collision[body_id]) {
      color_to_collisions[color].push_back(collision_idx);
    }
  }

  // Remove duplicates in each group (since a collision might be inserted twice)
  std::vector<std::vector<size_t>> collision_groups;
  for (auto &[_, group] : color_to_collisions) {
    std::sort(group.begin(), group.end());
    group.erase(std::unique(group.begin(), group.end()), group.end());
    collision_groups.push_back(std::move(group));
  }

  // Optional: sort groups by size (biggest first)
  std::sort(collision_groups.begin(), collision_groups.end(),
            [](const auto &a, const auto &b) { return a.size() > b.size(); });

  return collision_groups;
}
