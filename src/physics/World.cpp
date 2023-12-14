#include "World.hpp"

using namespace robosim;

World::World(scalar timestep, uint substeps){
    this->timestep;
    this->substeps = substeps;
} ;


void World::step(){
    scalar h =  this->timestep/ this->substeps;
    scalar inv_h = 1/h;

    for (int i=0; i<this->substeps; i++){
        // this->.narrow_phase_collision()
        this->update_bodies_position_and_orientation(h);
        this->solve_positions(inv_h);
        this->update_bodies_velocities(inv_h);
        this->solve_velocities(h);
    }
}

void World::update_bodies_position_and_orientation(scalar h){
    for (Body &body : this->bodies){
        body.update_position_and_orientation(h);
    }
}

void World::update_bodies_velocities(scalar inv_h){
    for (Body &body : this->bodies){
        body.update_velocities(inv_h);
    }
}

void World::solve_positions(scalar inv_h){
    for (PositionalConstraint constraint : this->positional_constraints){
        constraint.apply_constraint(inv_h);
    }
    for (RotationalConstraint constraint : this->rotational_constraints){
        constraint.apply_constraint(inv_h);
    }
}
void World::solve_velocities(scalar h){}

scalar World::get_time_step(){
    return this->timestep;
} 

void World::set_gravity(vec3 gravity){
    for (auto &body : this->bodies){
        body.set_gravity(gravity);
    }
}

uint World::add_body(Body body){
    this->bodies.push_back(body);

    return (uint)(this->bodies.size() - 1);
}

vec3 World::get_body_position(uint id){
    return this->bodies[id].position;
}

quat World::get_body_orientation(uint id){
    return this->bodies[id].orientation;
}