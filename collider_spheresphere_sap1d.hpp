#ifndef COLLIDER_SPHERESPHERE_SAP1D
#define COLLIDER_SPHERESPHERE_SAP1D
#include <utility>
#include "collider_spheresphere_base.hpp"
#include "container_typedef.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ColliderSphereSphereSAP1D : public ColliderSphereSphereBase
{
    /*

    Sphere-sphere broad phase collision checking via sweep and prune.
    Sphere radii are slightly enlarged so that collision checking does not need to occur frequently.

    Variables
    =========
    spheregroup_in : SphereGroup
        Spheres where collision checks are applied.
    axis_in : int
        Axis along which sweep and prune is performed.
        Use 0, 1, or 2 for the x, y, or z axes, respectively.

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

    // axis along which to sweep and prune
    int axis = 0;

    // vector of collision pairs
    std::vector<std::pair<int, int>> collision_vec;

    // functions
    std::vector<std::pair<int, int>> get_collision_vec();
    void update_collision_vec(int ts);

    // default constructor
    ColliderSphereSphereSAP1D() {}

    // constructor
    ColliderSphereSphereSAP1D(SphereGroup &spheregroup_in, int axis_in)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        axis = axis_in;

    }

    private:

    // functions
    bool is_aabb_collision(const SphereSphereAABB &aabb_i, const SphereSphereAABB &aabb_j);

};

std::vector<std::pair<int, int>> ColliderSphereSphereSAP1D::get_collision_vec()
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

void ColliderSphereSphereSAP1D::update_collision_vec(int ts)
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

    // check if update is necessary
    // skip if verlet distance is not yet traveled
    bool is_beyond_distance_verlet = false;
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        is_beyond_distance_verlet = sphere.distance_traveled > sphere.distance_verlet;
    }
    if (!is_beyond_distance_verlet)
    {
        return;
    }

    // clear collision vector
    collision_vec.clear();

    // initialize vector of AABB
    std::vector<SphereSphereAABB> aabb_vec;

    // iterate through each sphere
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        
        // reset traveled distances
        sphere.distance_traveled = 0.;

        // compute AABB
        SphereSphereAABB aabb_sub;
        aabb_sub.gid = sphere.gid;
        aabb_sub.x_min = sphere.position(0) - sphere.radius - sphere.distance_verlet;
        aabb_sub.y_min = sphere.position(1) - sphere.radius - sphere.distance_verlet;
        aabb_sub.z_min = sphere.position(2) - sphere.radius - sphere.distance_verlet;
        aabb_sub.x_max = sphere.position(0) + sphere.radius + sphere.distance_verlet;
        aabb_sub.y_max = sphere.position(1) + sphere.radius + sphere.distance_verlet;
        aabb_sub.z_max = sphere.position(2) + sphere.radius + sphere.distance_verlet;

        // store AABB
        aabb_vec.push_back(aabb_sub);

    }

    // sort AABBs according to min x, y, or z
    switch (axis)
    {
        case 0: std::sort(aabb_vec.begin(), aabb_vec.end(), [](const SphereSphereAABB& aabb_i, const SphereSphereAABB& aabb_j) {return aabb_i.x_min < aabb_j.x_min;}); break;
        case 1: std::sort(aabb_vec.begin(), aabb_vec.end(), [](const SphereSphereAABB& aabb_i, const SphereSphereAABB& aabb_j) {return aabb_i.y_min < aabb_j.y_min;}); break;
        case 2: std::sort(aabb_vec.begin(), aabb_vec.end(), [](const SphereSphereAABB& aabb_i, const SphereSphereAABB& aabb_j) {return aabb_i.z_min < aabb_j.z_min;}); break;
    }
    
    // list of "active" (potentially overlapping) AABBs
    std::vector<SphereSphereAABB> aabb_active_vec;

    // Sweep through the sorted boxes
    for (const auto& aabb : aabb_vec)
    {

        // remove boxes from the active list that are no longer overlapping in the x-axis
        switch (axis)
        {
            case 0: aabb_active_vec.erase(std::remove_if(aabb_active_vec.begin(), aabb_active_vec.end(), [&](const SphereSphereAABB& aabb_active) {return aabb_active.x_max < aabb.x_min;}), aabb_active_vec.end()); break;
            case 1: aabb_active_vec.erase(std::remove_if(aabb_active_vec.begin(), aabb_active_vec.end(), [&](const SphereSphereAABB& aabb_active) {return aabb_active.y_max < aabb.y_min;}), aabb_active_vec.end()); break;
            case 2: aabb_active_vec.erase(std::remove_if(aabb_active_vec.begin(), aabb_active_vec.end(), [&](const SphereSphereAABB& aabb_active) {return aabb_active.z_max < aabb.z_min;}), aabb_active_vec.end()); break;
        }
        
        // check for collisions with the remaining active AABBs
        for (const auto& aabb_active : aabb_active_vec){
        if (is_aabb_collision(aabb, aabb_active)){
            collision_vec.emplace_back(aabb_active.gid, aabb.gid); // Store the colliding pair
        }}

        // add the current AABB to the active list
        aabb_active_vec.push_back(aabb);
    
    }

}

bool ColliderSphereSphereSAP1D::is_aabb_collision(const SphereSphereAABB &aabb_i, const SphereSphereAABB &aabb_j)
{
    
    bool is_collision = (
        (aabb_i.x_min < aabb_j.x_max && aabb_i.x_max > aabb_j.x_min)  &&  // overlap along x
        (aabb_i.y_min < aabb_j.y_max && aabb_i.y_max > aabb_j.y_min)  &&  // overlap along y
        (aabb_i.z_min < aabb_j.z_max && aabb_i.z_max > aabb_j.z_min)      // overlap along z
    );
    
    return is_collision;

}

}

#endif
