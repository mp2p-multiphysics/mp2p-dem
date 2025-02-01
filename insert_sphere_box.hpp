#ifndef INSERT_SPHERE_BOX
#define INSERT_SPHERE_BOX
#include <algorithm>
#include <cstdlib>
#include <random>
#include <vector>
#include "insertdelete_base.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class InsertSphereBox : public InsertDeleteBase
{
    /*

    Inserts spheres from a CSV file at the start of the simulation.

    Variables
    =========
    spheregroup_in : SphereGroup
        Sphere group to be inserted to.
    mid_in : int
        Material ID.
    radius_in : double
        Radius of spheres to insert.
    num_spheres_in : int
        Number of spheres to insert.
    position_min_in : Eigen::Vector3d
        Minimum position of the region to insert spheres.
    position_max_in : Eigen::Vector3d
        Maximum position of the region to insert spheres.
    ts_start_in : int
        Timestep at which particle insertion starts.
    ts_end_in : int
        Timestep at which particle insertion ends.
        No insertion is made at and after this timestep.
        Set to -1 for no end.
        Default value is -1.
    ts_step_in : int
        Time interval between insertions.
        Set to -1 to insert particles only once.
        Default value is -1.
    radius_ratio_in : double
        Expands the region by this factor multipled by the radius of each sphere.
        Can be set to 0 to insert spheres whose centers are inside the region.
        Can be set to +1 to insert spheres that partially overlap with the region.
        Can be set to -1 to insert spheres that are completely outside the region.
        Default value is 0.
    velocity_in : Eigen::Vector3d
        Initial velocity of the spheres.
        Default value is {0, 0, 0}.
    num_attempt_max_in : int
        Maximum number of attempts to insert spheres.
        Default value is 100000.
    seed_in : int
        Seed for random number generator.
        Default value is 0.

    */

    public:

    // sphere group
    double dt = 0.;
    SphereGroup* spheregroup_ptr;

    // insertion parameters
    VectorInt mid_vec;
    VectorDouble radius_vec;
    VectorInt num_spheres_vec;
    EigenVector3D velocity = EigenVector3D::Zero();
    int num_attempt_max = 0;
    int seed = 0;

    // region
    EigenVector3D position_min = EigenVector3D::Zero();
    EigenVector3D position_max = EigenVector3D::Zero();
    double radius_ratio = 0.;

    // timesteps
    int ts_start = 0;
    int ts_end = 0;
    int ts_step = 0;

    // random number generator
    std::mt19937 rng;

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    InsertSphereBox() {}

    // constructor (one type of particle)
    InsertSphereBox(
        SphereGroup &spheregroup_in, int mid_in, double radius_in, int num_spheres_in,
        EigenVector3D position_min_in, EigenVector3D position_max_in,
        int ts_start_in, int ts_end_in = -1, int ts_step_in = -1, double radius_ratio_in = 0,
        EigenVector3D velocity_in = {0., 0., 0.}, int num_attempt_max_in = 100000, int seed_in = 0
    )
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        mid_vec.push_back(mid_in);
        radius_vec.push_back(radius_in);
        num_spheres_vec.push_back(num_spheres_in);

        // regions
        position_min = position_min_in;
        position_max = position_max_in;
        radius_ratio = radius_ratio_in;

        // timesteps
        ts_start = ts_start_in;
        ts_end = ts_end_in;
        ts_step = ts_step_in;

        // store other optional inputs
        velocity = velocity_in;
        num_attempt_max = num_attempt_max_in;
        seed = seed_in;

        // initialize random number generator
        rng = std::mt19937(seed);

    }

    // constructor (multiple types of particles)
    InsertSphereBox(
        SphereGroup &spheregroup_in, VectorInt mid_vec_in, VectorDouble radius_vec_in, VectorInt num_spheres_vec_in,
        EigenVector3D position_min_in, EigenVector3D position_max_in,
        int ts_start_in, int ts_end_in = -1, int ts_step_in = -1, double radius_ratio_in = 0,
        EigenVector3D velocity_in = {0., 0., 0.}, int num_attempt_max_in = 100000, int seed_in = 0
    )
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        mid_vec = mid_vec_in;
        radius_vec = radius_vec_in;
        num_spheres_vec = num_spheres_vec_in;

        // regions
        position_min = position_min_in;
        position_max = position_max_in;
        radius_ratio = radius_ratio_in;

        // timesteps
        ts_start = ts_start_in;
        ts_end = ts_end_in;
        ts_step = ts_step_in;

        // store other optional inputs
        velocity = velocity_in;
        num_attempt_max = num_attempt_max_in;
        seed = seed_in;

        // initialize random number generator
        rng = std::mt19937(seed);

    }

    private:

    // function
    bool check_overlap(const Sphere &sphere_test, const std::vector<Sphere, Eigen::aligned_allocator<Sphere>> &sphere_vec);

};

void InsertSphereBox::update(int ts)
{
    /*

    Updates this object.

    Arguments
    =========
    ts : int
        Timestep number.
    
    Returns
    =======
    (none)

    */

    // check if deletion is needed
    if (ts < ts_start)  // insertion has not started yet
    {
        return;  
    }
    if (ts_end != -1 && ts >= ts_end)  // delete has ended
    {
        return;  
    }
    if (ts_step == -1)
    {
        if (ts != ts_start)  // only delete once at ts_start_in
        {
            return;  
        }
    }
    else if ((ts - ts_start) % ts_step != 0)  // delete only at defined intervals
    {
        return;
    }

    // initialize
    int num_attempt = 0;
    int num_insert = 0;
    std::vector<Sphere, Eigen::aligned_allocator<Sphere>> sphere_vec;
    VectorInt index_vec; // index of mid_vec and radius_vec to use
    
    // generate random vector of indices
    // if num_spheres_vec is {A, B, ...}, index_vec will be {A number of 0, B number of 1, ...}
    for (int indx_i = 0; indx_i < num_spheres_vec.size(); indx_i++)
    {
        index_vec.insert(index_vec.end(), num_spheres_vec[indx_i], indx_i);
    }
    std::shuffle(index_vec.begin(), index_vec.end(), rng);  // shuffle index_vec

    // random number generator
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // get total number of spheres
    int num_spheres_all = 0;
    for (auto num_spheres : num_spheres_vec)
    {
        num_spheres_all += num_spheres;
    }

    // insert spheres
    while (sphere_vec.size() < num_spheres_all && num_attempt < num_attempt_max)
    {
        
        // create sphere
        Sphere sphere_sub;
        double radius = radius_vec[index_vec[num_insert]];
        sphere_sub.mid = mid_vec[index_vec[num_insert]];
        sphere_sub.radius = radius;

        // randomize position
        sphere_sub.position.coeffRef(0) = (position_min.coeffRef(0) - radius*radius_ratio) + dist(rng) * (position_max.coeffRef(0) - position_min.coeffRef(0) + 2*radius*radius_ratio);
        sphere_sub.position.coeffRef(1) = (position_min.coeffRef(1) - radius*radius_ratio) + dist(rng) * (position_max.coeffRef(1) - position_min.coeffRef(1) + 2*radius*radius_ratio);
        sphere_sub.position.coeffRef(2) = (position_min.coeffRef(2) - radius*radius_ratio) + dist(rng) * (position_max.coeffRef(2) - position_min.coeffRef(2) + 2*radius*radius_ratio);
        sphere_sub.velocity = velocity;
        sphere_sub.previous_position = sphere_sub.position - velocity * dt;

        // insert sphere if no overlap
        if (check_overlap(sphere_sub, sphere_vec))
        {
            sphere_vec.push_back(sphere_sub);
            num_insert++;
        }
        num_attempt++;

    }

    // append spheres to sphere group
    for (auto sphere : sphere_vec)
    {
        sphere.gid = spheregroup_ptr->num_sphere_max;
        spheregroup_ptr->tid_to_gid_vec.push_back(spheregroup_ptr->num_sphere_max);
        spheregroup_ptr->gid_to_tid_map[spheregroup_ptr->num_sphere_max] = spheregroup_ptr->num_sphere;
        spheregroup_ptr->sphere_vec.push_back(sphere);
        spheregroup_ptr->num_sphere++;
        spheregroup_ptr->num_sphere_max++;
    }

}

bool InsertSphereBox::check_overlap(const Sphere &sphere_test, const std::vector<Sphere, Eigen::aligned_allocator<Sphere>> &sphere_vec)
{
    
    for (const auto& sphere : sphere_vec)
    {
        if ((sphere_test.position - sphere.position).norm() < (sphere_test.radius + sphere.radius))
        {
            return false;
        }
    }

    return true;

}

}

#endif
