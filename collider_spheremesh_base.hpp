#ifndef COLLIDER_SPHEREMESH_BASE
#define COLLIDER_SPHEREMESH_BASE
#include "container_typedef.hpp"
#include "group_mesh.hpp"
#include "group_sphere.hpp"

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
    get_spheregroup_ptr : SphereGroup*
        Returns a pointer to the sphere group.
    get_meshgroup_ptr : MeshGroup*
        Returns a pointer to the mesh group.
    initialize : void
        Initializes the collider.
    update : void
        Updates the collider.

    */

    public:

    // functions
    virtual std::vector<std::pair<int, int>> get_collision_vec() {return {};};
    virtual SphereGroup* get_spheregroup_ptr() {return {};};
    virtual MeshGroup* get_meshgroup_ptr() {return {};};
    virtual void initialize(double dt_in) {};
    virtual void update(int ts) {};

    // default constructor
    ColliderSphereMeshBase() {}

    private:

};

}

#endif
