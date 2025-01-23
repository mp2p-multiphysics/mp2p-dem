#ifndef COLLIDER_SPHEREMESH_NAIVE
#define COLLIDER_SPHEREMESH_NAIVE
#include "container_typedef.hpp"
#include "group_mesh.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ColliderSphereMeshNaive
{

    public:

    // sphere and mesh group
    SphereGroup* spheregroup_ptr;
    MeshGroup* meshgroup_ptr;

    // vector of collision pairs
    std::vector<std::pair<int, int>> collision_vec;

    // functions
    std::vector<std::pair<int, int>> get_collision_vec();
    void update_collision_vec();

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
    return collision_vec;
}


void ColliderSphereMeshNaive::update_collision_vec()
{

    // clear vector
    collision_vec.clear();

    // iterate through each sphere combination
    for (int indx_i = 0; indx_i < spheregroup_ptr->num_sphere; indx_i++){
    for (int indx_j = 0; indx_j < meshgroup_ptr->num_mesh; indx_j++){

        // append to collision vector
        collision_vec.push_back({indx_i, indx_j});

    }}

}

}

#endif
