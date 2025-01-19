#ifndef COLLIDER_SPHERESPHERE_NAIVE
#define COLLIDER_SPHERESPHERE_NAIVE
#include <utility>
#include "collider_spheresphere_base.hpp"
#include "container_typedef.hpp"
#include "sphere.hpp"
#include "sphere_group.hpp"

namespace DEM
{

class ColliderSphereSphereNaive : public ColliderSphereSphereBase
{

    public:

    // vector of sphere groups
    std::vector<GroupSphere*> spheregroup_ptr_vec;

    // vector of collision pairs
    std::vector<std::pair<Sphere, Sphere>> collision_vec;

    // functions
    std::vector<std::pair<Sphere, Sphere>> get_collision_vec();
    void update_collision_vec();

    // default constructor
    ColliderSphereSphereNaive() {}

    // constructor
    ColliderSphereSphereNaive(std::vector<GroupSphere*> spheregroup_ptr_vec_in)
    {

        // store inputs
        spheregroup_ptr_vec = spheregroup_ptr_vec_in;

    }

    private:

};

std::vector<std::pair<Sphere, Sphere>> ColliderSphereSphereNaive::get_collision_vec()
{
    return collision_vec;
}

void ColliderSphereSphereNaive::update_collision_vec()
{

    // clear vector
    collision_vec.clear();

    // iterate through each sphere group combination
    for (auto spheregroup_ptr_i : spheregroup_ptr_vec){
    for (auto spheregroup_ptr_j : spheregroup_ptr_vec){

        // get number of spheres
        int num_sphere_i = spheregroup_ptr_i->sphere_vec.size();
        int num_sphere_j = spheregroup_ptr_j->sphere_vec.size();

        // check if sphere group i and j are different
        int is_spheregroup_different = int(spheregroup_ptr_i != spheregroup_ptr_j);

        // iterate through each sphere combination
        // j starts from i if sphere groups are the same
        // j starts from 0 if sphere groups are different
        for (int indx_i = 0; indx_i < num_sphere_i; indx_i++){
        for (int indx_j = indx_i * is_spheregroup_different; indx_j < num_sphere_j; indx_j++){

            // subset spheres
            Sphere sphere_i = spheregroup_ptr_i->sphere_vec[indx_i];
            Sphere sphere_j = spheregroup_ptr_j->sphere_vec[indx_j];

            // append to collision vector
            collision_vec.push_back({sphere_i, sphere_j});

        }}

    }}

}

}

#endif
