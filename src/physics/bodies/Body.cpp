
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