#ifndef COLLIDER_SPHEREMESH_NAIVE
#define COLLIDER_SPHEREMESH_NAIVE
#include "collider_spheremesh_base.hpp"
#include "container_typedef.hpp"
#include "group_mesh.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ColliderSphereMeshNaive : public ColliderSphereMeshBase
{
    /*

    Sphere-mesh broad phase collision checking via naive pairwise search.

    Variables
    =========
    spheregroup_in : SphereGroup
        Spheres where collision checks are applied.
    meshgroup_in : MeshGroup
        Meshes where collision checks are applied.

    */

    public:

    // sphere and mesh group
    SphereGroup* spheregroup_ptr;
    MeshGroup* meshgroup_ptr;

    // vector of collision pairs
    std::vector<std::pair<int, int>> collision_vec;

    // functions
    std::vector<std::pair<int, int>> get_collision_vec();
    SphereGroup* get_spheregroup_ptr() {return spheregroup_ptr;};
    MeshGroup* get_meshgroup_ptr() {return meshgroup_ptr;};
    void initialize(double dt_in) {};
    void update(int ts);

    // default constructor
    ColliderSphereMeshNaive() {}

    // constructor
    ColliderSphereMeshNaive(SphereGroup &spheregroup_in, MeshGroup &meshgroup_in)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        meshgroup_ptr = &meshgroup_in;

    }

    private:

};

std::vector<std::pair<int, int>> ColliderSphereMeshNaive::get_collision_vec()
{
    /*

    Returns (sphere group ID, mesh group ID) pairs that may collide.

    Arguments
    =========
    (none)

    Returns
    =======
    collision_vec : vector<pair<int, int>>
        Vector of (sphere group ID, mesh group ID) pairs that may collide.

    */

    return collision_vec;

}


void ColliderSphereMeshNaive::update(int ts)
{
    /*

    Updates the collision vector.

    Arguments
    =========
    ts : int
        Timestep number.

    Returns
    =======
    (none)

    */

    // clear collision vector
    collision_vec.clear();

    // iterate through each sphere combination
    for (int indx_i = 0; indx_i < spheregroup_ptr->num_sphere; indx_i++){
    for (int indx_j = 0; indx_j < meshgroup_ptr->num_mesh; indx_j++){

        // append to collision vector
        collision_vec.push_back({spheregroup_ptr->tid_to_gid_vec[indx_i], indx_j});

    }}

}

}

#endif
