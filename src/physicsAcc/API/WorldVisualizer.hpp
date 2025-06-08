

#pragma once

#include "VisualizerNew.hpp"
#include "physicsAcc/API/CollidersAPI.hpp"
#include <hpp/fcl/math/transform.h>
#include <raylib.h>

/**
 * @brief Struct to be used as a wrapper for the visualizer and the world
 * */
struct WorldVisualizer {
  rbvs::Visualizer visualizer;
  std::map<size_t, rbvs::Entity> collider_to_vis_shape;

  WorldVisualizer() : visualizer(1920, 1080, "RoboVis : Visualizer") {}

  /**
   * @brief General function to update the colliders
   */
  void update(const ColliderCollection &colliders) {
    this->update_colliders(colliders);
    this->visualizer.update();
  }

  /**
   * @brief Updates the collider positions on the simulation
   */
  void update_colliders(const ColliderCollection &colliders) {
    for (size_t collider_id = 0; collider_id < colliders.n_colliders;
         collider_id++) {
      hpp::fcl::Transform3f tf =
          colliders.collider[collider_id]->getTransform();
      vec3 p = ti::from_eigen(tf.getTranslation());
      quat q = ti::from_eigen(tf.getQuatRotation());
      rbvs::ModelUpdateParams update_params;
      update_params.orientation = ti::to_raylib(q);
      update_params.position = ti::to_raylib(p);
      update_params.entity = this->collider_to_vis_shape.at(collider_id);
      this->visualizer.update_model(update_params);
    }
  }

  /**
   * @brief Adds a collider to the
   */
  void add_collider(const ColliderParams params, size_t collider_id,
                    const BodyCollection &bodies) {

    rbvs::ModelParams model_params;

    switch (params.model_type) {
    case ColliderShape::CONE: {
      model_params.color = GREEN;
      model_params.model_type = rbvs::ModelType::CONE;
      model_params.radius = params.radius;
      model_params.length = params.length;
      break;
    }
    case ColliderShape::CYLINDER: {
      model_params.color = PURPLE;
      model_params.model_type = rbvs::ModelType::CYLINDER;
      model_params.radius = params.radius;
      model_params.length = params.length;
      break;
    }
    case ColliderShape::BOX: {
      model_params.color = RED;
      model_params.model_type = rbvs::ModelType::BOX;
      model_params.half_extents = {static_cast<float>(params.half_extents.x),
                                   static_cast<float>(params.half_extents.y),
                                   static_cast<float>(params.half_extents.z)};
      break;
    }
    case ColliderShape::SPHERE: {
      model_params.color = BLUE;
      model_params.model_type = rbvs::ModelType::SPHERE;
      model_params.radius = params.radius;
      break;
    }
    case ColliderShape::PLANE: {
      model_params.color = WHITE;
      model_params.model_type = rbvs::ModelType::BOX;
      // Implementation pending on RoboVis
      std::cerr << "PLANE COLLIDER NOT IMPLEMENTED RoboVis Yet\n";
      break;
    }
    case ColliderShape::CONVEX_MESH: {
      model_params.color = ORANGE;
      model_params.model_type = rbvs::ModelType::MESH;
      model_params.model_path = params.model_path;
      break;
    }
    default: {
      std::cerr << "NOT A VALID COLLIDER SHAPER WAS SELECTED\n";
      break;
    }
    }

    vec3 body_pos = bodies.position[params.body_id];
    quat body_rot = bodies.orientation[params.body_id];

    vec3 world_position = body_pos + ti::rotate(body_rot, params.position);
    quat world_orientation = body_rot * params.orientation;
    model_params.position = ti::to_raylib(world_position);
    model_params.orientation = ti::to_raylib(world_orientation);
    rbvs::Entity vis_entity = this->visualizer.create_model(model_params);
    this->collider_to_vis_shape.insert({collider_id, vis_entity});
  }
  // void add_visual_shape(const rbvs::ModelParams, size_t body_id) {}
};
