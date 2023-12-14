#include "StaticBody.hpp"
#include "physics/math/math.hpp"

StaticBody::StaticBody(
    vec3 position,
    quat orientation) : 
    Body(position, orientation)
{
    // Set the type
    this->type = BodyType::STATIC;
}

StaticBody::~StaticBody()
{
}
