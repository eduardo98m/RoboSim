#include "World.hpp"

using namespace robosim;

size_t World::create_articulated_system(
    const std::vector<size_t> &joint_ids,
    const std::vector<JointType> &joint_types,
    const std::vector<size_t> &link_ids)
{

    ArticulatedSystem articulated_system = {.joint_ids = joint_ids,
                                            .joint_types = joint_types,
                                            .link_ids = link_ids};

    this->articulated_systems.push_back(articulated_system);

    return this->articulated_systems.size() - 1;
};

std::vector<scalar> World::get_articulated_system_state(size_t id){
    ArticulatedSystem &articulated_system = this->articulated_systems[id];

    std::vector<scalar> state;
    
    // Base position
    vec3 base_position = this->get_body_position(articulated_system.link_ids[0]);
    state.push_back(base_position.x);
    state.push_back(base_position.y);
    state.push_back(base_position.z);

    // Base orientation
    quat base_orientation = this->get_body_orientation(articulated_system.link_ids[0]); 
    state.push_back(base_orientation.x);
    state.push_back(base_orientation.y);
    state.push_back(base_orientation.z);
    state.push_back(base_orientation.w);

    for (int i =0; i < articulated_system.joint_ids.size(); i++){

        size_t joint_id  = articulated_system.joint_ids[i];
        switch (articulated_system.joint_types[i])
        {
        case JointType::HINGE:{
            scalar  angle = this->revolute_joint_constraints[joint_id]->get_current_angle();
            state.push_back(angle);
            break;
        }
        case JointType::PRISMATIC:{
            scalar  position = this->prismatic_joint_constraints[joint_id]->get_current_position();
            state.push_back(position);
            break;
        }      
        default:
            break;
        }
    }

    return state;
     
}

pose World::get_articulated_system_link_pose(size_t id,  size_t link_id){
    
    ArticulatedSystem &articulated_system = this->articulated_systems[id];

    size_t body_id = articulated_system.link_ids[link_id];

    return pose{
        .position = this->get_body_position(body_id),
        .orientation = this->get_body_orientation(body_id)
    };
}

void World::set_articulated_system_joint_targets(size_t id, std::vector<scalar> joint_targets){
    ArticulatedSystem &articulated_system = this->articulated_systems[id];

    assert(joint_targets.size() == articulated_system.joint_ids.size());

    for (size_t i = 0; i < joint_targets.size(); i++){
        size_t joint_id = articulated_system.joint_ids[i];
        scalar target  = joint_targets[i];

        
        switch (articulated_system.joint_types[i])
        {
        case JointType::HINGE:{
            switch (this->revolute_joint_constraints[joint_id]->type)
            {
            case JointControlType::DRIVEN_BY_POSITION:
                this->set_revolute_joint_target_angle(joint_id, target);
                break;
            case JointControlType::DRIVEN_BY_SPEED:
                this->set_revolute_joint_target_speed(joint_id, target);
                break;
            default:
                break;
            }
            break;
        }
        case JointType::PRISMATIC:{
            switch (this->prismatic_joint_constraints[joint_id]->type)
            {
            case JointControlType::DRIVEN_BY_POSITION:
                this->set_prismatic_joint_target_position(joint_id, target);
                break;
            case JointControlType::DRIVEN_BY_SPEED:
                this->set_prismatic_joint_target_speed(joint_id, target);
                break;
            default:
                break;
            }
            break;
        }
        default:
            break;
        }
    }

}