#ifndef COLLISIONCHECK_SPHEREWALLMESH_NAIVE
#define COLLISIONCHECK_SPHEREWALLMESH_NAIVE
#include <utility>
#include <vector>
#include "container_sphere.hpp"
#include "container_typedef.hpp"
#include "container_wallmesh.hpp"

class CollisionCheckSphereWallMeshNaive
{

    public:

    // functions
    VectorPairInt broad_search(SpherePositionVelocityStruct &sphere_pvs, WallMeshPositionVelocityStruct &wallmesh_pvs);

    // default constructor
    CollisionCheckSphereWallMeshNaive()
    {

    }

};

VectorPairInt CollisionCheckSphereWallMeshNaive::broad_search(SpherePositionVelocityStruct &sphere_pvs, WallMeshPositionVelocityStruct &wallmesh_pvs)
{

    // initialize vector with index pairs
    VectorPairInt collision_vec;

    // iterate through particle i
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++){
    for (int indx_k = 0; indx_k < wallmesh_pvs.num_mesh; indx_k++){

        // add collision pair
        std::pair<int, int> pair_sub = {indx_i, indx_k};
        collision_vec.push_back(pair_sub);

    }}

    return collision_vec;

}

#endif
