#ifndef COLLIDER_SPHEREMESH_NAIVE
#define COLLIDER_SPHEREMESH_NAIVE
#include <utility>
#include "collider_spheremesh_base.hpp"
#include "container_typedef.hpp"
#include "particle_sphere.hpp"

namespace DEM
{

class ColliderSphereMeshNaive : public ColliderSphereMeshBase
{

    public:
    
    // vector of sphere and mesh objects
    std::vector<ParticleSphere*> sphere_ptr_vec;
    std::vector<WallMesh*> mesh_ptr_vec;
    std::vector<std::pair<ParticleSphere*, WallMesh*>> spheremesh_ptr_pair_vec;

    // functions used in timestepping
    void compute_collide_pair(CollisionSphereMeshVector &collision_vec, int ts);

    // default constructor
    ColliderSphereMeshNaive() {}

    // constructor
    ColliderSphereMeshNaive(std::vector<ParticleSphere*> sphere_ptr_vec_in, std::vector<WallMesh*> mesh_ptr_vec_in)
    {

        // store vector of spheres and meshes
        sphere_ptr_vec = sphere_ptr_vec_in;
        mesh_ptr_vec = mesh_ptr_vec_in;

        // generate pairs of sphere objects
        generate_spheremesh_ptr_pair();

    }

    private:

    void generate_spheremesh_ptr_pair();

};

void ColliderSphereMeshNaive::compute_collide_pair(CollisionSphereMeshVector &collision_vec, int ts)
{

    // iterate through each pair
    for (auto spheremesh_ptr_pair : spheremesh_ptr_pair_vec)
    {

        // subset sphere and mesh
        ParticleSphere* sphere_ptr_i = spheremesh_ptr_pair.first;
        WallMesh* mesh_ptr_j = spheremesh_ptr_pair.second;

        // iterate through each combination
        for (int indx_i = 0; indx_i < sphere_ptr_i->num_element; indx_i++){
        for (int indx_j = 0; indx_j < mesh_ptr_j->num_element; indx_j++){

            // get permanent IDs
            int pid_i = sphere_ptr_i->tid_to_pid_vec[indx_i];
            int pid_j = indx_j;
            
            // create pair of colliding objects
            std::pair<ParticleSphere*, int> sphere_pid_i = {sphere_ptr_i, pid_i};
            std::pair<WallMesh*, int> mesh_pid_j = {mesh_ptr_j, pid_j};

            // append to vector
            collision_vec.push_back({sphere_pid_i, mesh_pid_j});

        }}

    }

}

void ColliderSphereMeshNaive::generate_spheremesh_ptr_pair()
{

    // get number of sphere objects
    int num_sphere = sphere_ptr_vec.size();

    // iterate through all combinations (including self-combination)
    for (int indx_i = 0; indx_i < num_sphere; indx_i++){
    for (int indx_j = 0; indx_j < num_sphere; indx_j++){

        // insert pair of sphere objects to vector
        std::pair<ParticleSphere*, WallMesh*> spheremesh_pair = {sphere_ptr_vec[indx_i], mesh_ptr_vec[indx_j]};
        spheremesh_ptr_pair_vec.push_back(spheremesh_pair);

    }}

}

}

#endif
