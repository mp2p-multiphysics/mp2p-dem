#ifndef COLLIDER_SPHEREMESH_SAP1D
#define COLLIDER_SPHEREMESH_SAP1D
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>
#include "collider_spheremesh_base.hpp"
#include "container_typedef.hpp"
#include "group_mesh.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ColliderSphereMeshSAP1D : public ColliderSphereMeshBase
{
    /*

    Sphere-mesh broad phase collision checking via sweep and prune.
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
    For spheres, the verlet distance is distance_verlet_abs + distance_verlet_rel * (radius).
    For meshes, the verlet distance is distance_verlet_abs + distance_verlet_rel * (radius of the largest sphere).

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // sphere and mesh group
    SphereGroup* spheregroup_ptr;
    MeshGroup* meshgroup_ptr;

    // vector of collision pairs
    std::vector<std::pair<int, int>> collision_vec;

    // sweep and prune parameters
    int axis = 0;  // axis along which sweep and prune is performed
    double distance_verlet_abs = 0;  // absolute verlet distance
    double distance_verlet_rel = 0;  // verlet distance relative to sphere radius
    std::unordered_map<int, double> sphere_distance_verlet_map;  // sphere verlet distance
    std::unordered_map<int, double> mesh_distance_verlet_map;  // mesh verlet distance
    std::unordered_map<int, EigenVector3D> sphere_position_previous_map;  // sphere position at previous collision check
    std::unordered_map<int, EigenVector3D> mesh_position_p0_previous_map;  // mesh p0 position at previous collision check
    std::unordered_map<int, EigenVector3D> mesh_position_p1_previous_map;  // mesh p1 position at previous collision check
    std::unordered_map<int, EigenVector3D> mesh_position_p2_previous_map;  // mesh p2 position at previous collision check

    // previous number of spheres
    int num_sphere_max_previous = 0;
    int num_sphere_previous = 0;

    // functions
    std::vector<std::pair<int, int>> get_collision_vec();
    SphereGroup* get_spheregroup_ptr() {return spheregroup_ptr;};
    MeshGroup* get_meshgroup_ptr() {return meshgroup_ptr;};
    void initialize(double dt_in);
    void update(int ts);

    // default constructor
    ColliderSphereMeshSAP1D() {}

    // constructor
    ColliderSphereMeshSAP1D(SphereGroup &spheregroup_in, MeshGroup &meshgroup_in, int axis_in, double distance_verlet_abs_in = 0., double distance_verlet_rel_in = 0.5)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        meshgroup_ptr = &meshgroup_in;
        axis = axis_in;

        // optional inputs
        distance_verlet_abs = distance_verlet_abs_in;
        distance_verlet_rel = distance_verlet_rel_in;

    }

    private:

    // functions
    bool is_aabb_collision(const SphereMeshAABB &aabb_i, const SphereMeshAABB &aabb_j);

};

std::vector<std::pair<int, int>> ColliderSphereMeshSAP1D::get_collision_vec()
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


void ColliderSphereMeshSAP1D::initialize(double dt_in)
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
    double sphere_radius_max = 0;
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        sphere_distance_verlet_map[sphere.gid] = distance_verlet_abs + distance_verlet_rel * sphere.radius;
        sphere_position_previous_map[sphere.gid] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};  // triggers the first collision check
        sphere_radius_max = std::max(sphere_radius_max, sphere.radius);
    }
    for (auto &mesh : meshgroup_ptr->mesh_vec)
    {
        mesh_distance_verlet_map[mesh.gid] = distance_verlet_abs + distance_verlet_rel * sphere_radius_max;
        mesh_position_p0_previous_map[mesh.gid] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};  // triggers the first collision check
        mesh_position_p1_previous_map[mesh.gid] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};  // triggers the first collision check
        mesh_position_p2_previous_map[mesh.gid] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};  // triggers the first collision check
    }

    // store previous number of spheres
    int num_sphere_max_previous = spheregroup_ptr->num_sphere_max;
    int num_sphere_previous = spheregroup_ptr->num_sphere;

}

void ColliderSphereMeshSAP1D::update(int ts)
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

        // recalculate verlet distance and previous position
        double sphere_radius_max = 0;
        for (auto &sphere : spheregroup_ptr->sphere_vec)
        {
            sphere_distance_verlet_map[sphere.gid] = distance_verlet_abs + distance_verlet_rel * sphere.radius;
            sphere_radius_max = std::max(sphere_radius_max, sphere.radius);
        }
        for (auto &mesh : meshgroup_ptr->mesh_vec)
        {
            mesh_distance_verlet_map[mesh.gid] = distance_verlet_abs + distance_verlet_rel * sphere_radius_max;
        }

    } 

    // rerun collider if any sphere has traveled beyond verlet distance
    if (!is_rerun_collider){
    for (auto &mesh : meshgroup_ptr->mesh_vec){  // mesh

        // get verlet distance and last position
        double distance_verlet = mesh_distance_verlet_map[mesh.gid];
        EigenVector3D position_previous_p0 = mesh_position_p0_previous_map[mesh.gid];
        EigenVector3D position_previous_p1 = mesh_position_p1_previous_map[mesh.gid];
        EigenVector3D position_previous_p2 = mesh_position_p2_previous_map[mesh.gid];

        // check if mesh has traveled beyond verlet distance
        double distance_traveled_p0 = (mesh.position_p0 - position_previous_p0).norm();
        double distance_traveled_p1 = (mesh.position_p1 - position_previous_p1).norm();
        double distance_traveled_p2 = (mesh.position_p2 - position_previous_p2).norm();
        
        // stop checking if any mesh point has traveled beyond verlet distance
        is_rerun_collider = distance_traveled_p0 > distance_verlet || distance_traveled_p1 > distance_verlet || distance_traveled_p2 > distance_verlet;
        if (is_rerun_collider)
        {
            break;
        }

    }}
    if (!is_rerun_collider){
    for (auto &sphere : spheregroup_ptr->sphere_vec)  // sphere
    {
        
        // get verlet distance and last position
        double distance_verlet = sphere_distance_verlet_map[sphere.gid];
        EigenVector3D position_previous = sphere_position_previous_map[sphere.gid];

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
        sphere_position_previous_map[sphere.gid] = sphere.position;
    }
    for (auto &mesh : meshgroup_ptr->mesh_vec)
    {
        mesh_position_p0_previous_map[mesh.gid] = mesh.position_p0;
        mesh_position_p1_previous_map[mesh.gid] = mesh.position_p1;
        mesh_position_p2_previous_map[mesh.gid] = mesh.position_p2;
    }

    // initialize vector of AABB
    std::vector<SphereMeshAABB> aabb_vec;

    // iterate through each sphere
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        
        // compute AABB
        SphereMeshAABB aabb_sub;
        aabb_sub.gid = sphere.gid;
        aabb_sub.is_sphere = true;
        aabb_sub.x_min = sphere.position(0) - sphere.radius - sphere_distance_verlet_map[sphere.gid];
        aabb_sub.y_min = sphere.position(1) - sphere.radius - sphere_distance_verlet_map[sphere.gid];
        aabb_sub.z_min = sphere.position(2) - sphere.radius - sphere_distance_verlet_map[sphere.gid];
        aabb_sub.x_max = sphere.position(0) + sphere.radius + sphere_distance_verlet_map[sphere.gid];
        aabb_sub.y_max = sphere.position(1) + sphere.radius + sphere_distance_verlet_map[sphere.gid];
        aabb_sub.z_max = sphere.position(2) + sphere.radius + sphere_distance_verlet_map[sphere.gid];

        // store AABB
        aabb_vec.push_back(aabb_sub);

    }

    // iterate through each mesh
    for (auto &mesh : meshgroup_ptr->mesh_vec)
    {
        
        // compute AABB
        SphereMeshAABB aabb_sub;
        EigenVector3D pos_min = (mesh.position_p0).cwiseMin(mesh.position_p1).cwiseMin(mesh.position_p2);
        EigenVector3D pos_max = (mesh.position_p0).cwiseMax(mesh.position_p1).cwiseMax(mesh.position_p2);
        aabb_sub.gid = mesh.gid;
        aabb_sub.is_sphere = false;
        aabb_sub.x_min = pos_min(0) - mesh_distance_verlet_map[mesh.gid];
        aabb_sub.y_min = pos_min(1) - mesh_distance_verlet_map[mesh.gid];
        aabb_sub.z_min = pos_min(2) - mesh_distance_verlet_map[mesh.gid];
        aabb_sub.x_max = pos_max(0) + mesh_distance_verlet_map[mesh.gid];
        aabb_sub.y_max = pos_max(1) + mesh_distance_verlet_map[mesh.gid];
        aabb_sub.z_max = pos_max(2) + mesh_distance_verlet_map[mesh.gid];

        // store AABB
        aabb_vec.push_back(aabb_sub);

    }

    // sort AABBs according to min x, y, or z
    switch (axis)
    {
        case 0: std::sort(aabb_vec.begin(), aabb_vec.end(), [](const SphereMeshAABB& aabb_i, const SphereMeshAABB& aabb_j) {return aabb_i.x_min < aabb_j.x_min;}); break;
        case 1: std::sort(aabb_vec.begin(), aabb_vec.end(), [](const SphereMeshAABB& aabb_i, const SphereMeshAABB& aabb_j) {return aabb_i.y_min < aabb_j.y_min;}); break;
        case 2: std::sort(aabb_vec.begin(), aabb_vec.end(), [](const SphereMeshAABB& aabb_i, const SphereMeshAABB& aabb_j) {return aabb_i.z_min < aabb_j.z_min;}); break;
    }
    
    // list of "active" (potentially overlapping) AABBs
    std::vector<SphereMeshAABB> aabb_active_vec;

    // Sweep through the sorted boxes
    for (const auto& aabb : aabb_vec)
    {

        // remove boxes from the active list that are no longer overlapping in the x-axis
        switch (axis)
        {
            case 0: aabb_active_vec.erase(std::remove_if(aabb_active_vec.begin(), aabb_active_vec.end(), [&](const SphereMeshAABB& aabb_active) {return aabb_active.x_max < aabb.x_min;}), aabb_active_vec.end()); break;
            case 1: aabb_active_vec.erase(std::remove_if(aabb_active_vec.begin(), aabb_active_vec.end(), [&](const SphereMeshAABB& aabb_active) {return aabb_active.y_max < aabb.y_min;}), aabb_active_vec.end()); break;
            case 2: aabb_active_vec.erase(std::remove_if(aabb_active_vec.begin(), aabb_active_vec.end(), [&](const SphereMeshAABB& aabb_active) {return aabb_active.z_max < aabb.z_min;}), aabb_active_vec.end()); break;
        }
        
        // check for collisions with the remaining active AABBs
        for (const auto& aabb_active : aabb_active_vec){
        if ((aabb.is_sphere != aabb_active.is_sphere) && is_aabb_collision(aabb, aabb_active)){
            
            // store collision pair (sphere group ID, mesh group ID)
            if (aabb.is_sphere)
            {
                collision_vec.emplace_back(aabb.gid, aabb_active.gid);
            }
            else
            {
                collision_vec.emplace_back(aabb_active.gid, aabb.gid);
            }

        }}

        // add the current AABB to the active list
        aabb_active_vec.push_back(aabb);
    
    }

}

bool ColliderSphereMeshSAP1D::is_aabb_collision(const SphereMeshAABB &aabb_i, const SphereMeshAABB &aabb_j)
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
