#ifndef COLLIDER_SPHERESPHERE_NAIVE
#define COLLIDER_SPHERESPHERE_NAIVE
#include <utility>
#include "collider_spheresphere_base.hpp"
#include "container_typedef.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ColliderSphereSphereNaive : public ColliderSphereSphereBase
{
    /*

    Sphere-sphere broad phase collision checking via naive pairwise search.

    Variables
    =========
    spheregroup_in : SphereGroup
        Spheres where collision checks are applied.

    Functions
    =========
    get_collision_vec : vector<pair<int, int>>
        Returns (sphere group ID, sphere group ID) pairs that may collide.
    update_collision_vec : void
        Updates the collision vector.

    */

    public:

    // sphere group
    SphereGroup* spheregroup_ptr;

    // vector of collision pairs
    std::vector<std::pair<int, int>> collision_vec;

    // functions
    std::vector<std::pair<int, int>> get_collision_vec();
    void update_collision_vec();

    // default constructor
    ColliderSphereSphereNaive() {}

    // constructor
    ColliderSphereSphereNaive(SphereGroup &spheregroup_in)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;

    }

    private:

};

std::vector<std::pair<int, int>> ColliderSphereSphereNaive::get_collision_vec()
{
    /*

    Returns (sphere group ID, sphere group ID) pairs that may collide.

    Arguments
    =========
    (none)

    Returns
    =======
    collision_vec : vector<pair<int, int>>
        Vector of (sphere group ID, sphere group ID) pairs that may collide.

    Notes
    =====
    The collision vector outputs combinations and not permutations.

    */


    return collision_vec;

}

void ColliderSphereSphereNaive::update_collision_vec()
{

    // clear vector
    collision_vec.clear();

    // iterate through each sphere combination
    for (int indx_i = 0; indx_i < spheregroup_ptr->num_sphere; indx_i++){
    for (int indx_j = indx_i + 1; indx_j < spheregroup_ptr->num_sphere; indx_j++){

        // append to collision vector
        collision_vec.push_back({indx_i, indx_j});

    }}

}

}

#endif
