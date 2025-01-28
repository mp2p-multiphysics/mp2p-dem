#ifndef COLLIDER_SPHEREMESH_SAP1D
#define COLLIDER_SPHEREMESH_SAP1D
#include "collider_spheremesh_base.hpp"
#include "container_typedef.hpp"
#include "group_mesh.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ColliderSphereMeshSAP1D : public ColliderSphereMeshBase
{
    /*

    Sphere-mesh broad phase collision checking via naive pairwise search.

    Variables
    =========
    spheregroup_in : SphereGroup
        Spheres where collision checks are applied.
    meshgroup_in : MeshGroup
        Meshes where collision checks are applied.
    axis_in : int
        Axis along which sweep and prune is performed.
        Use 0, 1, or 2 for the x, y, or z axes, respectively.

    Functions
    =========
    get_collision_vec : vector<pair<int, int>>
        Returns (sphere group ID, mesh index) pairs that may collide.
    update_collision_vec : void
        Updates the collision vector.

    */

    public:

    // sphere and mesh group
    SphereGroup* spheregroup_ptr;
    MeshGroup* meshgroup_ptr;

    // axis along which to sweep and prune
    int axis = 0;

    // vector of collision pairs
    std::vector<std::pair<int, int>> collision_vec;

    // functions
    std::vector<std::pair<int, int>> get_collision_vec();
    void update_collision_vec(int ts);

    // default constructor
    ColliderSphereMeshSAP1D() {}

    // constructor
    ColliderSphereMeshSAP1D(SphereGroup &spheregroup_in, MeshGroup &meshgroup_in, int axis_in)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        meshgroup_ptr = &meshgroup_in;
        axis = axis_in;

    }

    private:

    // functions
    bool is_aabb_collision(const SphereMeshAABB &aabb_i, const SphereMeshAABB &aabb_j);

};

std::vector<std::pair<int, int>> ColliderSphereMeshSAP1D::get_collision_vec()
{
    /*

    Returns (sphere group ID, mesh index) pairs that may collide.

    Arguments
    =========
    (none)

    Returns
    =======
    collision_vec : vector<pair<int, int>>
        Vector of (sphere group ID, mesh index) pairs that may collide.

    */

    return collision_vec;

}

void ColliderSphereMeshSAP1D::update_collision_vec(int ts)
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
    for (auto &mesh : meshgroup_ptr->mesh_vec)
    {
        is_beyond_distance_verlet = (
            mesh.distance_traveled_p0 > mesh.distance_verlet ||
            mesh.distance_traveled_p1 > mesh.distance_verlet ||
            mesh.distance_traveled_p2 > mesh.distance_verlet
        );   
    }
    if (!is_beyond_distance_verlet)
    {
        return;
    }

    // clear collision vector
    collision_vec.clear();

    // initialize vector of AABB
    std::vector<SphereMeshAABB> aabb_vec;

    // iterate through each sphere
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        
        // reset traveled distances
        sphere.distance_traveled = 0.;

        // compute AABB
        SphereMeshAABB aabb_sub;
        aabb_sub.gid = sphere.gid;
        aabb_sub.is_sphere = true;
        aabb_sub.x_min = sphere.position(0) - sphere.radius - sphere.distance_verlet;
        aabb_sub.y_min = sphere.position(1) - sphere.radius - sphere.distance_verlet;
        aabb_sub.z_min = sphere.position(2) - sphere.radius - sphere.distance_verlet;
        aabb_sub.x_max = sphere.position(0) + sphere.radius + sphere.distance_verlet;
        aabb_sub.y_max = sphere.position(1) + sphere.radius + sphere.distance_verlet;
        aabb_sub.z_max = sphere.position(2) + sphere.radius + sphere.distance_verlet;

        // store AABB
        aabb_vec.push_back(aabb_sub);

    }

    // iterate through each mesh
    for (auto &mesh : meshgroup_ptr->mesh_vec)
    {
        
        // reset traveled distances
        mesh.distance_traveled_p0 = 0.;
        mesh.distance_traveled_p1 = 0.;
        mesh.distance_traveled_p2 = 0.;

        // compute AABB
        SphereMeshAABB aabb_sub;
        EigenVector3D pos_min = (mesh.position_p0).cwiseMin(mesh.position_p1).cwiseMin(mesh.position_p2);
        EigenVector3D pos_max = (mesh.position_p0).cwiseMax(mesh.position_p1).cwiseMax(mesh.position_p2);
        aabb_sub.gid = mesh.gid;
        aabb_sub.is_sphere = false;
        aabb_sub.x_min = pos_min(0) - mesh.distance_verlet;
        aabb_sub.y_min = pos_min(1) - mesh.distance_verlet;
        aabb_sub.z_min = pos_min(2) - mesh.distance_verlet;
        aabb_sub.x_max = pos_max(0) + mesh.distance_verlet;
        aabb_sub.y_max = pos_max(1) + mesh.distance_verlet;
        aabb_sub.z_max = pos_max(2) + mesh.distance_verlet;

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
        if (aabb.is_sphere xor aabb_active.is_sphere && is_aabb_collision(aabb, aabb_active)){
            collision_vec.emplace_back(aabb_active.gid, aabb.gid); // Store the colliding pair
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
