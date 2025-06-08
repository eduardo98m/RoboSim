#pragma once
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include "hpp/fcl/broadphase/broadphase_collision_manager.h"

#include "hpp/fcl/BVH/BVH_model.h"
#include "hpp/fcl/collision.h"
#include "hpp/fcl/collision_data.h"

#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <queue>


#include "physics/math/math.hpp"
#include "physicsAcc/Constraints/Contact.hpp"
#include "physicsAcc/Body/BodyCollection.hpp"

struct ColliderUserData
{
    size_t body_id;
    int collision_group;
    scalar restitution = 0.5;
    scalar static_friction = 0.5;
    scalar dynamic_friction = 0.5;
    vec3 position = {0.0, 0.0, 0.0};         // Realtive position of the collider with respect to its  body
    quat orientation = {1.0, 0.0, 0.0, 0.0}; // Relative position of the scalar with respect to its  body
};

/**
 * @brief
 * @param n_colliders : The number of colliders in the collection
 * @param colliders: Vector of colliders
 * @param body_id :  The body id to which the collider is asscosiated with
 */
struct ColliderCollection
{
    size_t n_colliders = 0;
    std::vector<std::shared_ptr<hpp::fcl::CollisionObject>> collider = {};
    hpp::fcl::DynamicAABBTreeArrayCollisionManager collision_manager;
};

/**
 * @brief Result of the broad phase collision detection step
 * @param n_possible_collisions Number of detected (possible) collision pairs.
 * @param collider_1 List of the first collider of the pair
 * @param collider_2 List of the second collider of the pair.
 */
struct BroadPhaseResult
{
    size_t n_possible_collisions = 0;
    std::vector<size_t> collider_1;
    std::vector<size_t> collider_2;
};

/**
 * @brief Sets the pose of a collider within a ColliderCollection.
 *
 * This function updates the position and orientation of a specific collider
 * within a given ColliderCollection. It also updates the collision manager
 * to reflect the change.
 *
 * @param cc The ColliderCollection containing the collider to update.
 * @param i The index of the collider within the ColliderCollection to update.
 * @param position The desired new position of the collider.
 * @param orientation The desired new orientation of the collider.
 * @return True if the pose was successfully set; false if the index is out of bounds.
 */
bool set_collider_pose(ColliderCollection &cc, size_t i, const vec3 &position, const quat &orientation);

struct BroadPhaseCallBack : hpp::fcl::CollisionCallBackBase
{

    BroadPhaseResult result;

    void init()
    {
        return;
    }

    bool collide(hpp::fcl::CollisionObject *objA, hpp::fcl::CollisionObject *objB)
    {
        ColliderUserData *dataA = static_cast<ColliderUserData *>(objA->getUserData());
        ColliderUserData *dataB = static_cast<ColliderUserData *>(objB->getUserData());
        if (dataA && dataB)
        {

            if (dataA->body_id == dataB->body_id)
                return true;
            // TODO : Add the collision group filtering
            // TODO:  Fix this to avoid false contacts between fixed bodies (performance improvement)
            result.collider_1.push_back(dataA->body_id);
            result.collider_2.push_back(dataB->body_id);
            result.n_possible_collisions++;
        }
        else
        {
            std::cerr << "Error: Collision object missing user data in broadphase.\n";
        }

        // Return true to continue checking other pairs, false to stop.
        return true;
    }
};

/**
 * @brief Updates the poses of all colliders in a ColliderCollection based on the associated bodies' positions and orientations.
 *
 * This function iterates through each collider in the provided ColliderCollection.
 * For each collider, it retrieves the corresponding body's position and orientation
 * from the BodyCollection and updates the collider's pose accordingly. It also
 * updates the collision manager to reflect these changes.  Static bodies are skipped.
 *
 * @param cc The ColliderCollection to update.
 * @param bc The BodyCollection containing the bodies associated with the colliders.
 * @param timestep The time step used for the simulation.  Used to expand the AABB for fast moving objects.
 */
void update_collider_poses(ColliderCollection &cc, const BodyCollection &bc, scalar timestep);

/**
 * @brief Performs broad-phase collision detection on a ColliderCollection.
 *
 * This function uses a broad-phase collision detection algorithm (Dynamic AABB Tree)
 * to identify potential collision pairs within the given ColliderCollection. It returns
 * a BroadPhaseResult struct containing the indices of the potentially colliding colliders.
 *
 * @param cc The ColliderCollection to perform collision detection on.
 * @return A BroadPhaseResult struct containing the results of the broad-phase collision detection.
 */
BroadPhaseResult broad_phase_collision_detection(ColliderCollection &cc);

/**
 * @brief Solves a contact constraint between two bodies at the position level.
 *
 * This function applies position correction to resolve interpenetration between two
 * colliding bodies. It updates the positions of the bodies based on the contact
 * normal and penetration depth. It also updates the poses of the colliders associated with the bodies.
 *
 * @param contact_collection The collection of contact constraints.
 * @param contact_id The index of the contact constraint to solve.
 * @param cc The ColliderCollection containing the colliders associated with the bodies.
 * @param collider_a The index of the first collider.
 * @param collider_b The index of the second collider.
 * @param bc The BodyCollection containing the bodies.
 * @param body_a The index of the first body.
 * @param body_b The index of the second body.
 * @param inverse_time_step The inverse of the time step used for the simulation.
 */
void solve_contact(ContactCollection &contact_collection,
                   size_t contact_id,
                   ColliderCollection &cc,
                   size_t collider_a,
                   size_t collider_b,
                   BodyCollection &bc,
                   size_t body_a,
                   size_t body_b,
                   scalar inverse_time_step);

/**
 * @brief Performs narrow-phase collision detection between potentially colliding pairs of bodies.
 *
 * This function iterates through the pairs of potentially colliding bodies identified by
 * the broad-phase collision detection. For each pair, it performs a more accurate
 * collision detection using the hpp-fcl library to generate contact points.  It then solves
 * the contact constraint at the position level.
 *
 * @param cc The ColliderCollection containing the colliders associated with the bodies.
 * @param bc The BodyCollection containing the bodies.
 * @param broad_phase_pairs The BroadPhaseResult struct containing the pairs of potentially colliding bodies.
 * @param inverse_time_step The inverse of the time step used for the simulation.
 * @return A ContactCollection containing the contact constraints generated by the narrow-phase collision detection.
 */
ContactCollection narrow_phase_collision(ColliderCollection &cc,
                                         BodyCollection &bc,
                                         const BroadPhaseResult &broad_phase_pairs,
                                         scalar inverse_time_step);