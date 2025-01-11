#ifndef COLLIDER_SPHEREMESH_BASE
#define COLLIDER_SPHEREMESH_BASE
#include <utility>
#include <vector>
#include "container_spheremesh.hpp"
#include "particle_sphere.hpp"
#include "wall_mesh.hpp"

namespace DEM
{

class ColliderSphereMeshBase
{

    public:

    // functions used in timestepping
    void compute_collide_pair(CollisionSphereMeshVector &collision_vec, int ts) {};

    // default constructor
    ColliderSphereMeshBase() {}

    private:

};

}

#endif
