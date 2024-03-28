#pragma once
#include <vector>


enum JointType{
    HINGE,
    PRISMATIC,
    SPHERICAL,
    FIXED
};

struct ArticulatedSystem
{   
    std::vector<size_t> joint_ids;
    std::vector<JointType> joint_types;
    std::vector<size_t> link_ids;


    // ArticulatedSystem(const std::vector<size_t> &joint_ids, 
    //                   const std::vector<JointType> &joint_types,
    //                   const std::vector<size_t> &link_ids
    //                   )
    //                   {
    //                     this->joint_ids = joint_ids;
    //                     this->joint_types = joint_types;
    //                     this->link_ids = link_ids;
    //                   }
    
};

