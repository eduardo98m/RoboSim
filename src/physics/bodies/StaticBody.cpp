#include "StaticBody.hpp"
#include "physics/math/math.hpp"

StaticBody::StaticBody(
    vec3 position,
    quat orientation) : 
    Body(position, orientation)
{
}

StaticBody::~StaticBody()
{
}

void StaticBody::apply_positional_constraint_impulse(vec3 impulse, vec3 r)
{
    // Do nothing
}