#ifndef COLLIDER_SPHERESPHERE_SAP1D
#define COLLIDER_SPHERESPHERE_SAP1D
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>
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
    meshgroup_in : MeshGroup
        Meshes where collision checks are applied.
    axis_in : int
        Axis along which sweep and prune is performed.
        Use 0, 1, or 2 for the x, y, or z axes, respectively.
    distance_verlet_abs_in : double
        Absolute component of the verlet distance.
    distance_verlet_rel_in : double
        Relative component of the verlet distance.

    Notes
    =====
    The verlet distance is distance_verlet_abs + distance_verlet_rel * radius.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // sphere group
    SphereGroup* spheregroup_ptr;

    // vector of collision pairs
    std::vector<std::pair<int, int>> collision_vec;

    // sweep and prune parameters
    int axis = 0;  // axis along which sweep and prune is performed
    double distance_verlet_abs = 0;  // absolute verlet distance
    double distance_verlet_rel = 0;  // verlet distance relative to sphere radius
    std::unordered_map<int, double> distance_verlet_map;  // verlet distance for each sphere
    std::unordered_map<int, EigenVector3D> position_previous_map;  // position at previous collision check

    // previous number of spheres
    int num_sphere_max_previous = 0;
    int num_sphere_previous = 0;

    // functions
    std::vector<std::pair<int, int>> get_collision_vec();
    SphereGroup* get_spheregroup_ptr() {return spheregroup_ptr;};
    void initialize(double dt_in);
    void update(int ts);

    // default constructor
    ColliderSphereSphereSAP1D() {}

    // constructor
    ColliderSphereSphereSAP1D(SphereGroup &spheregroup_in, int axis_in, double distance_verlet_abs_in = 0., double distance_verlet_rel_in = 0.5)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        axis = axis_in;

        // optional inputs
        distance_verlet_abs = distance_verlet_abs_in;
        distance_verlet_rel = distance_verlet_rel_in;

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

void ColliderSphereSphereSAP1D::initialize(double dt_in)
{
    /*

    Initializes the collider.

    Arguments
    =========
    dt_in : double
        Duration of timestep.

    Returns
    =======
    (none)

    */

    // initialize verlet distance and previous position (current position)
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        distance_verlet_map[sphere.gid] = distance_verlet_abs + distance_verlet_rel * sphere.radius;
        position_previous_map[sphere.gid] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};  // triggers the first collision check
    }

    // store previous number of spheres
    int num_sphere_max_previous = spheregroup_ptr->num_sphere_max;
    int num_sphere_previous = spheregroup_ptr->num_sphere;

}

void ColliderSphereSphereSAP1D::update(int ts)
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

    // rerun collider if particles have been inserted or deleted
    bool is_rerun_collider = false;
    if (spheregroup_ptr->num_sphere_max != num_sphere_max_previous || spheregroup_ptr->num_sphere != num_sphere_previous)
    {
        
        // rerun collider
        is_rerun_collider = true;

        // update previous number of spheres
        num_sphere_max_previous = spheregroup_ptr->num_sphere_max;
        num_sphere_previous = spheregroup_ptr->num_sphere;

        // recalculate verlet distance if spheres were added or removed
        for (auto &sphere : spheregroup_ptr->sphere_vec)
        {
            distance_verlet_map[sphere.gid] = distance_verlet_abs + distance_verlet_rel * sphere.radius;
        }

    } 

    // rerun collider if any sphere has traveled beyond verlet distance
    if (!is_rerun_collider){
    for (auto &sphere : spheregroup_ptr->sphere_vec){
        
        // get verlet distance and last position
        double distance_verlet = distance_verlet_map[sphere.gid];
        EigenVector3D position_previous = position_previous_map[sphere.gid];

        // check if sphere has traveled beyond verlet distance
        double distance_traveled = (sphere.position - position_previous).norm();
        
        // stop checking if any sphere has traveled beyond verlet distance
        is_rerun_collider = distance_traveled > distance_verlet;
        if (is_rerun_collider)
        {
            break;
        }

    }}
    if (!is_rerun_collider)
    {
        return;
    }

    // clear collision vector
    collision_vec.clear();

    // record previous position
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        position_previous_map[sphere.gid] = sphere.position;
    }

    // initialize vector of AABB
    std::vector<SphereSphereAABB> aabb_vec;

    // iterate through each sphere
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        
        // compute AABB
        SphereSphereAABB aabb_sub;
        aabb_sub.gid = sphere.gid;
        aabb_sub.x_min = sphere.position(0) - sphere.radius - distance_verlet_map[sphere.gid];
        aabb_sub.y_min = sphere.position(1) - sphere.radius - distance_verlet_map[sphere.gid];
        aabb_sub.z_min = sphere.position(2) - sphere.radius - distance_verlet_map[sphere.gid];
        aabb_sub.x_max = sphere.position(0) + sphere.radius + distance_verlet_map[sphere.gid];
        aabb_sub.y_max = sphere.position(1) + sphere.radius + distance_verlet_map[sphere.gid];
        aabb_sub.z_max = sphere.position(2) + sphere.radius + distance_verlet_map[sphere.gid];

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

            // ensure that the pair is unique (first < second)
            if (aabb.gid > aabb_active.gid)
            {
                collision_vec.emplace_back(aabb_active.gid, aabb.gid);
            }
            else
            {
                collision_vec.emplace_back(aabb.gid, aabb_active.gid);
            }

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
