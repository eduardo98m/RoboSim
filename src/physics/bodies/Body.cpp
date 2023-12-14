
#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"

Body::Body(
    vec3 position,
    quat orientation
    )
{
    this->position = position;
    this->orientation = orientation;
}

Body::~Body()
{
}

void Body::apply_positional_constraint_impulse(vec3 impulse, vec3 r){}
void Body::apply_rotational_constraint_impulse(vec3 impulse){}


scalar Body::get_positional_generalized_inverse_mass(vec3 r, vec3 n){return 0.0;}
scalar Body::get_rotational_generalized_inverse_mass(vec3 n) {return 0.0;}