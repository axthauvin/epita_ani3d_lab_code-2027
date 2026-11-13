#include "squash.hpp"

using namespace cgp;

void squash_compute_offset(squash_structure const& squash, numarray<vec3> const& bind_position, numarray<vec3>& offset)
{
    float front = bind_position[0].z;
    for (vec3 const& p : bind_position)
        front = std::max(front, p.z); // Tom looks toward +z in the bind pose

    offset.resize(bind_position.size());
    for (int k = 0; k < bind_position.size(); ++k)
        offset[k] = vec3(0, 0, squash.flatness * squash.amount * (front - bind_position[k].z)); // moves the vertex to the front (flattenisation of the character)
}
