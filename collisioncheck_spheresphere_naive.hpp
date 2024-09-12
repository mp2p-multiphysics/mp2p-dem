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
    void set_input(VectorDouble &radius_vec_in);
    VectorPairInt broad_search(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    CollisionCheckSphereSphereNaive()
    {

    }

};

void CollisionCheckSphereSphereNaive::set_input(VectorDouble &radius_vec_in)
{
    // not needed for naive search
}

VectorPairInt CollisionCheckSphereSphereNaive::broad_search(SpherePositionVelocityStruct &sphere_pvs)
{

    // initialize vector with index pairs
    VectorPairInt collision_vec;

    // iterate through particle i
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++){
    for (int indx_j = indx_i + 1; indx_j < sphere_pvs.num_particle; indx_j++){

        // add collision pair
        std::pair<int, int> pair_sub = {indx_i, indx_j};
        collision_vec.push_back(pair_sub);

    }}

    return collision_vec;

}

#endif
