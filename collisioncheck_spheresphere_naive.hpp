#ifndef COLLISIONCHECK_SPHERESPHERE_NAIVE
#define COLLISIONCHECK_SPHERESPHERE_NAIVE
#include <utility>
#include <vector>
#include "container_sphere.hpp"
#include "container_typedef.hpp"

class CollisionCheckSphereSphereNaive
{

    public:

    // functions
    VectorPairInt broad_search(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    CollisionCheckSphereSphereNaive()
    {

    }

};

VectorPairInt CollisionCheckSphereSphereNaive::broad_search(SpherePositionVelocityStruct &sphere_pvs)
{

    // initialize vector with index pairs
    VectorPairInt collision_vec;

    // iterate through particle i
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++){
    for (int indx_j = 0; indx_j < sphere_pvs.num_particle; indx_j++){

        // skip self-collisions
        if (indx_i == indx_j)
        {
            continue;
        }

        // add collision pair
        std::pair<int, int> pair_sub = {indx_i, indx_j};
        collision_vec.push_back(pair_sub);

    }}

    return collision_vec;

}

#endif
