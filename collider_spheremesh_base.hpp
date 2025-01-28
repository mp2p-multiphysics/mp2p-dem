#ifndef COLLIDER_SPHEREMESH_BASE
#define COLLIDER_SPHEREMESH_BASE
#include "container_typedef.hpp"

namespace DEM
{

struct SphereMeshAABB
{

    // global ID
    int gid;

    // type of object
    bool is_sphere;

    // minimum point
    double x_min;
    double y_min;
    double z_min;

    // maximum point
    double x_max;
    double y_max;
    double z_max;

};

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
    virtual void update_collision_vec(int ts) {};

    // default constructor
    ColliderSphereMeshBase() {}

    private:

};

}

#endif
