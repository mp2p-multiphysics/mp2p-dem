#ifndef COLLIDER_SPHERESPHERE_BASE
#define COLLIDER_SPHERESPHERE_BASE
#include <utility>
#include <vector>
#include "container_spheresphere.hpp"
#include "particle_sphere.hpp"

namespace DEM
{

class ColliderSphereSphereBase
{

    public:

    // functions used in timestepping
    void compute_collide_pair(CollisionSphereSphereVector &collision_vec, int ts) {};

    // default constructor
    ColliderSphereSphereBase() {}

    private:

};

}

#endif
