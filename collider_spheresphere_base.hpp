#ifndef COLLIDER_SPHERESPHERE_BASE
#define COLLIDER_SPHERESPHERE_BASE
#include "container_typedef.hpp"
#include "group_sphere.hpp"

namespace DEM
{

struct SphereSphereAABB
{

    // global ID
    int gid;

    // minimum point
    double x_min;
    double y_min;
    double z_min;

    // maximum point
    double x_max;
    double y_max;
    double z_max;

};

class ColliderSphereSphereBase
{
    /*

    Base class for sphere-sphere broad phase collision checkers.

    Functions
    =========
    get_collision_vec : vector<pair<int, int>>
        Returns (sphere group ID, mesh index) pairs that may collide.
    get_spheregroup_ptr : SphereGroup*
        Returns a pointer to the sphere group.
    initialize : void
        Initializes the collider.
    update : void
        Updates the collider.

    */

    public:

    // functions
    virtual std::vector<std::pair<int, int>> get_collision_vec() {return {};};
    virtual SphereGroup* get_spheregroup_ptr() {return {};};
    virtual void initialize(double dt_in) {};
    virtual void update(int ts) {};

    // default constructor
    ColliderSphereSphereBase() {}

    private:

};

}

#endif
