#ifndef COLLIDER_SPHERESPHERE_NAIVE
#define COLLIDER_SPHERESPHERE_NAIVE
#include <utility>
#include "collider_spheresphere_base.hpp"
#include "particle_sphere.hpp"
#include "container_typedef.hpp"

namespace DEM
{

class ColliderSphereSphereNaive : public ColliderSphereSphereBase
{

    public:
    
    // vector of sphere objects
    std::vector<ParticleSphere*> sphere_ptr_vec;
    std::vector<std::pair<ParticleSphere*, ParticleSphere*>> spheresphere_ptr_pair_vec;

    // functions used in timestepping
    void compute_collide_pair(CollisionSphereSphereVector &collision_vec, int ts);

    // default constructor
    ColliderSphereSphereNaive() {}

    // constructor
    ColliderSphereSphereNaive(std::vector<ParticleSphere*> sphere_ptr_vec_in)
    {

        // store vector of spheres
        sphere_ptr_vec = sphere_ptr_vec_in;

        // generate pairs of sphere objects
        generate_spheresphere_ptr_pair();

    }

    private:

    void generate_spheresphere_ptr_pair();

};

void ColliderSphereSphereNaive::compute_collide_pair(CollisionSphereSphereVector &collision_vec, int ts)
{

    // iterate through each pair
    for (auto spheresphere_ptr_pair : spheresphere_ptr_pair_vec)
    {

        // subset spheres
        ParticleSphere* sphere_ptr_i = spheresphere_ptr_pair.first;
        ParticleSphere* sphere_ptr_j = spheresphere_ptr_pair.second;

        // verify if i and j are the same
        bool is_i_j_same = (sphere_ptr_i == sphere_ptr_j);

        // iterate through each combination
        // i and j are different - iterate through every pid_i and pid_j permutation
        // i and j are the same - iterate through every pid_i and pid_j combination
        for (int indx_i = 0; indx_i < sphere_ptr_i->num_element; indx_i++){
        for (int indx_j = indx_i * int(is_i_j_same); indx_j < sphere_ptr_j->num_element; indx_j++){

            // get permanent IDs
            int pid_i = sphere_ptr_i->tid_to_pid_vec[indx_i];
            int pid_j = sphere_ptr_j->tid_to_pid_vec[indx_j];
            
            // create pair of colliding objects
            std::pair<ParticleSphere*, int> sphere_pid_i = {sphere_ptr_i, pid_i};
            std::pair<ParticleSphere*, int> sphere_pid_j = {sphere_ptr_j, pid_j};

            // append to vector
            collision_vec.push_back({sphere_pid_i, sphere_pid_j});

        }}

    }

}

void ColliderSphereSphereNaive::generate_spheresphere_ptr_pair()
{

    // get number of sphere objects
    int num_sphere = sphere_ptr_vec.size();

    // iterate through all combinations (including self-combination)
    for (int indx_i = 0; indx_i < num_sphere; indx_i++){
    for (int indx_j = indx_i; indx_j < num_sphere; indx_j++){

        // insert pair of sphere objects to vector
        std::pair<ParticleSphere*, ParticleSphere*> spheresphere_pair = {sphere_ptr_vec[indx_i], sphere_ptr_vec[indx_j]};
        spheresphere_ptr_pair_vec.push_back(spheresphere_pair);

    }}

}

}

#endif
