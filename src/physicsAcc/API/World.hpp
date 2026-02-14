#pragma once
#include "BodyAPI.hpp"
#include "physics/math/math.hpp"
#include "physicsAcc/Body/BodyCollection.hpp"
#include "physicsAcc/Colliders/Collider.hpp"
#include "physicsAcc/Constraints/ConstraintCollection.hpp"
#include "physicsAcc/Constraints/Contact.hpp"
#include "physicsAcc/Constraints/Joint.hpp"

#include "physicsAcc/API/BodyAPI.hpp"
#include "physicsAcc/API/CollidersAPI.hpp"
#include "physicsAcc/API/ConstraintsAPI.hpp"
#include "physicsAcc/API/JointsAPI.hpp"

// Adding the visualizer torugh a custom API
#include "physicsAcc/API/WorldVisualizer.hpp"

struct World {
  scalar timestep = 1 / 60.;
  int substeps = 20;
  BodyCollection bodies;
  ConstraintCollection constraints;
  JointCollection joints;
  ContactCollection contacts;
  ColliderCollection colliders;
  std::shared_ptr<WorldVisualizer> visualizer;

  World(bool enable_visualizer = true)
      : bodies(), constraints(), joints(), contacts(), colliders() {
    if (enable_visualizer)
      visualizer = std::make_shared<WorldVisualizer>();
  }

  // Body Creation
  size_t create_body(BodyParams params) {
    size_t id = ::create_body(this->bodies, params);
    if (this->visualizer) {
      this->visualizer->add_body(params, id);
    }
    return id;
  }

  size_t create_collider(ColliderParams params) {
    size_t id = ::create_collider(this->colliders, params);
    if (this->visualizer) {
      this->visualizer->add_collider(params, id, bodies);
    }
    return id;
  }

  // Constraint Creation
  size_t create_constraint(ConstraintsParams params) {
    return ::create_constraint(this->constraints, params);
  }

  size_t create_prismatic_joint(PrismaticJointParams params) {
    return ::create_prismatic_joint(joints, constraints, params);
  }

  size_t create_revolute_joint(RevoluteJointParams params) {
    return ::create_revolute_joint(joints, constraints, params);
  }

  size_t create_fixed_joint(FixedJointParams params) {
    return ::create_fixed_joint(joints, constraints, params);
  }

  void solve_positions(const BroadPhaseResult &broad_phase_pairs, scalar inv_h,
                       scalar h) {
    // Contacts are created and solved  (The contacs are solved with projected
    // gauss-siedel)
    contacts =
        narrow_phase_collision(colliders, bodies, broad_phase_pairs, inv_h);

    // We solve the constraints using Jacobi (compute and then solve the errors)
    compute_joint_errors(joints, bodies, constraints, h);
    solve_constraints(bodies, constraints, inv_h);
  }

  void solve_velocities(scalar h) {
    solve_contacts_velocity_level(contacts, bodies, h);
    apply_joint_damping(joints, bodies, h);
  };

  void step() {
    scalar h = timestep / this->substeps;
    scalar inv_h = 1 / h;

    update_collider_poses(colliders, bodies, timestep);
    BroadPhaseResult broad_phase_pairs =
        broad_phase_collision_detection(colliders);

    for (int i = 0; i < this->substeps; i++) {
      update_position_and_orientation(bodies, h);
      this->solve_positions(broad_phase_pairs, inv_h, h);
      update_velocities(bodies, inv_h);
      this->solve_velocities(h);
    }

    // Visualizer Update
    if (this->visualizer) {
      this->visualizer->update(bodies, colliders);
    }
  };
};
