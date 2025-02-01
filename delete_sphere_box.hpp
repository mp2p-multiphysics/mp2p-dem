#ifndef DELETE_SPHERE_BOX
#define DELETE_SPHERE_BOX
#include "insertdelete_base.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class DeleteSphereBox : public InsertDeleteBase
{
    /*

    Deletes spheres inside a hexahedral region at specified times.

    Variables
    =========
    spheregroup_in : SphereGroup
        Sphere group to be deleted from.
    position_min_in : Eigen::Vector3d
        Minimum corner of the hexahedral region.
    position_max_in : Eigen::Vector3d
        Maximum corner of the hexahedral region.
    ts_start_in : int
        Timestep at which particle insertion starts.
        Default value is 0.
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
        Can be set to 0 to delete spheres whose centers are outside the region.
        Can be set to +1 to delete spheres that partially overlap with the region.
        Can be set to -1 to delete spheres that are completely inside the region.
        Default value is 0.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // sphere group
    double dt = 0.;
    SphereGroup* spheregroup_ptr;

    // region
    EigenVector3D position_min = EigenVector3D::Zero();
    EigenVector3D position_max = EigenVector3D::Zero();
    double radius_ratio = 0.;

    // timesteps
    int ts_start = 0;
    int ts_end = 0;
    int ts_step = 0;

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    DeleteSphereBox() {}

    // constructor
    DeleteSphereBox(
        SphereGroup &spheregroup_in, EigenVector3D position_min_in, EigenVector3D position_max_in,
        int ts_start_in = 0, int ts_end_in = -1, int ts_step_in = -1, double radius_ratio_in = 0
    )
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;

        // regions
        position_min = position_min_in;
        position_max = position_max_in;
        radius_ratio = radius_ratio_in;

        // timesteps
        ts_start = ts_start_in;
        ts_end = ts_end_in;
        ts_step = ts_step_in;

    }

    private:

};

void DeleteSphereBox::update(int ts)
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

    // iterate through each sphere
    for (auto it = spheregroup_ptr->sphere_vec.begin(); it != spheregroup_ptr->sphere_vec.end();)
    {

        // get sphere position
        EigenVector3D pos = it->position;

        // check if sphere is inside region
        double radius_sphere = it->radius;
        bool is_delete = (
            pos(0) < position_min(0) - radius_ratio*radius_sphere || pos(0) > position_max(0) + radius_ratio*radius_sphere ||  // x
            pos(1) < position_min(1) - radius_ratio*radius_sphere || pos(1) > position_max(1) + radius_ratio*radius_sphere ||  // y
            pos(2) < position_min(2) - radius_ratio*radius_sphere || pos(2) > position_max(2) + radius_ratio*radius_sphere     // z
        );

        // delete sphere from group
        if (is_delete)
        {
            spheregroup_ptr->num_sphere--;
            spheregroup_ptr->gid_to_tid_map.erase(it->gid);
            auto index = std::distance(spheregroup_ptr->sphere_vec.begin(), it);
            spheregroup_ptr->tid_to_gid_vec.erase(spheregroup_ptr->tid_to_gid_vec.begin() + index);
            for (size_t i = index; i < spheregroup_ptr->tid_to_gid_vec.size(); ++i)
            {
                spheregroup_ptr->gid_to_tid_map[spheregroup_ptr->tid_to_gid_vec[i]] = i;
            }
            it = spheregroup_ptr->sphere_vec.erase(it);  // returns next valid iterator
        }
        else
        {
            ++it;  // Only increment if not erased
        }

    }
    
}

}

#endif
