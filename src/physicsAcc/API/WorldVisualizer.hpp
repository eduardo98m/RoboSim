#pragma once

#include "API/Model.hpp"
#include "ECS/Entity.hpp"
#include "VisualizerNew.hpp"

#include "physicsAcc/API/BodyAPI.hpp"
#include "physicsAcc/API/CollidersAPI.hpp"
#include "physicsAcc/Body/BodyCollection.hpp"

#include <Eigen/src/Core/TriangularMatrix.h>
#include <hpp/fcl/math/transform.h>
#include <raylib.h>

/**
 * @brief Struct to be used as a wrapper for the visualizer and the world
 * */
struct WorldVisualizer {
  rbvs::Visualizer visualizer;
  std::map<size_t, rbvs::Entity> collider_to_vis_shape;
  std::map<size_t, rbvs::Entity> body_to_frame;

  WorldVisualizer() : visualizer(1920, 1080, "RoboVis : Visualizer") {
    this->register_refence_frame_model();

    // Directional light to test
    rbvs::Entity light =
        visualizer.create_light({.position = {0.0, 0.0, 2.0},
                                 .direction = {0.0, 0.0, -1.0},
                                 .type = rbvs::LightType::DIRECTIONAL,
                                 .intensity = 1.0});
    // rbvs::Entity light_point =
    //     visualizer.create_light({.position = {0.0, 0.0, 2.0},
    //                              .direction = {0.0, 0.0, -1.0},
    //                              .type = rbvs::LightType::POINT,
    //                              .intensity = 20.0});
  }

  /**
   * @brief General function to update the colliders
   */
  void update(const BodyCollection &bodies,
              const ColliderCollection &colliders) {
    this->update_colliders(colliders);
    this->update_bodies(bodies);
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
   * @brief Updates the collider model (It is not necessary to specify the
   * entity on the params)
   */
  void update_collider_model(size_t collider_id,
                             rbvs::ModelUpdateParams params) {
    params.entity = this->collider_to_vis_shape.at(collider_id);
    this->visualizer.update_model(params);
  }

  /**
   * @brief Updates the body reference frame model visuals (the user can change
   * the color and scale)
   */
  void update_body_frame_model(size_t body_id, rbvs::ModelUpdateParams params) {
    params.entity = this->body_to_frame.at(body_id);
    this->visualizer.update_model(params);
  }
  /**
   * @brief Updates the position of the body
   */
  void update_bodies(const BodyCollection &bodies) {
    for (size_t body_id = 0; body_id < bodies.n_bodies; ++body_id) {
      rbvs::ModelUpdateParams update_params;
      update_params.orientation = ti::to_raylib(bodies.orientation[body_id]);
      update_params.position = ti::to_raylib(bodies.position[body_id]);
      update_params.entity = this->body_to_frame.at(body_id);
      this->visualizer.update_model(update_params);
    }
  }

  void toggle_frame_visible_on_body(size_t body_id, bool toggle_visible) {
    rbvs::ModelUpdateParams update_params;
    update_params.visible = toggle_visible;
    update_params.entity = this->body_to_frame.at(body_id);
    this->visualizer.update_model(update_params);
  }

  void toogle_body_frames(bool toggle_visible) {
    for (auto const &pair : this->body_to_frame) {
      toggle_frame_visible_on_body(pair.first, toggle_visible);
    }
  }

  void show_reference_frames() {
    rbvs::Entity reference_frame_model = visualizer.create_model(
        rbvs::ModelParams{.position = {1.0, 1.0, 1.0},
                          .color = WHITE,
                          .model_type = rbvs::ModelType::CUSTOM,
                          .custom_model_key = "reference_frame"

        });
  }

  void register_refence_frame_model(void) {
    float total_lenght = 1.0;
    float tip_radius = 0.1;
    float cylinder_radius = tip_radius * 0.5;
    float cylinder_length = total_lenght * 0.8;
    float tip_height = total_lenght - cylinder_length;

    std::vector<rbvs::ModelPrimitive> reference_frame_primitives;

    Color colors[3] = {RED, GREEN, BLUE};

    Quaternion orientations[3] = {
        QuaternionFromEuler(0, 0, -PI / 2), // X axis
        QuaternionIdentity(),               // Y axis
        QuaternionFromEuler(PI / 2, 0, 0)   // Z axis
    };

    Vector3 axes[3] = {
        {1.0f, 0.0f, 0.0f}, // X
        {0.0f, 1.0f, 0.0f}, // Y
        {0.0f, 0.0f, 1.0f}  // Z
    };

    for (int i = 0; i < 3; i++) {
      // Position so that base of cylinder is at origin
      rbvs::ModelPrimitive cylinder = {.orientation = orientations[i],
                                       .type =
                                           rbvs::ModelPrimitiveType::CYLINDER,
                                       .radius = cylinder_radius,
                                       .height = cylinder_length,
                                       .color = colors[i]};

      // Position cone tip after cylinder
      Vector3 tip_pos = axes[i] * cylinder_length;
      rbvs::ModelPrimitive cone = {.position = tip_pos,
                                   .orientation = orientations[i],
                                   .type = rbvs::ModelPrimitiveType::CONE,
                                   .radius = tip_radius,
                                   .height = tip_height,
                                   .color = colors[i]};

      reference_frame_primitives.push_back(cylinder);
      reference_frame_primitives.push_back(cone);
    }

    reference_frame_primitives.push_back({.type = rbvs::SPHERE,
                                          .radius = cylinder_radius * 1.15f,
                                          .color = WHITE});

    this->visualizer.register_model("reference_frame",
                                    reference_frame_primitives);
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

  /**
   * @brief Adds all the visual elements for a body
   */
  void add_body(const BodyParams params, size_t body_id) {

    rbvs::Entity frame_entity = this->visualizer.create_model(
        {.position = ti::to_raylib(params.position),
         .orientation = ti::to_raylib(params.orientation),
         .color = WHITE,
         .model_type = rbvs::ModelType::CUSTOM,
         .custom_model_key = "reference_frame",
         .receive_lighting = false});

    this->body_to_frame[body_id] = frame_entity;

    this->toggle_frame_visible_on_body(body_id, false);
  }
};
