#ifndef COLLIDER_SPHEREMESH_BASE
#define COLLIDER_SPHEREMESH_BASE
#include "container_typedef.hpp"

namespace DEM
{

class ColliderSphereMeshBase
{
    /*

    Base class for sphere-mesh broad phase collision checkers.

    Functions
    =========
    get_collision_vec : vector<pair<int, int>>
        Returns (sphere group ID, mesh index) pairs that may collide.
    update_collision_vec : void
        Updates the collision vector.

    */

    public:

    // functions
    virtual std::vector<std::pair<int, int>> get_collision_vec() {return {};};
    virtual void update_collision_vec() {};

    // default constructor
    ColliderSphereMeshBase() {}

    private:

};

}

#endif
