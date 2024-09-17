#ifndef COLLISIONCHECK_SPHERESPHERE_NAIVE
#define COLLISIONCHECK_SPHERESPHERE_NAIVE
#include <utility>
#include <vector>
#include "container_sphere.hpp"
#include "container_typedef.hpp"

class CollisionCheckSphereSphereNaive
{
    /*

    Collision checker for sphere-sphere collisions.
    Uses naive pairwise search.

    Variables
    =========
    (none)

    Functions
    =========
    set_input : void
        Sets input variables to collision checker.
    broad_search : VectorPairInt
        Generates vector of sphere pairs that may collide.

    Notes
    ====
    This collision checker tests every possible pair for collision.
    This is inefficient and is provided only for debugging more efficient collision checkers.

    */


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
    /*

    Sets input variables to collision checker.

    Arguments
    =========
    radius_vec_in : VectorDouble
        vector with the radius of each type of sphere.

    Returns
    =======
    (none)

    */    
    
    // not needed for naive search

}

VectorPairInt CollisionCheckSphereSphereNaive::broad_search(SpherePositionVelocityStruct &sphere_pvs)
{
    /*

    Generates vector of sphere pairs that may collide.

    Arguments
    =========
    sphere_pvs : SpherePositionVelocityStruct
        struct with position and velocity of each sphere.

    Returns
    =======
    collision_vec : VectorPairInt
        vector of sphere pairs that may collide.

    */

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
