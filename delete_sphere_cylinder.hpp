#ifndef DELETE_SPHERE_CYLINDER
#define DELETE_SPHERE_CYLINDER
#include "insertdelete_base.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class DeleteSphereCylinder : public InsertDeleteBase
{
    /*

    Deletes spheres inside a cylindrical region at specified times.

    Variables
    =========
    spheregroup_in : SphereGroup
        Sphere group to be deleted from.
    axis_in : int
        Axis of the cylinder.
        0 for x-axis, 1 for y-axis, 2 for z-axis.
    position_center_in : Eigen::Vector3d
        Center of the cylinder.
    radius_in : double
        Radius of the cylinder.
    height_in : double
        Height of the cylinder.    
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
        Default value is 0, which deletes spheres whose centers are outside the region.
        Can be set to +1 to delete spheres that partially overlap with the region.
        Can be set to -1 to delete spheres that are completely inside the region.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // sphere group
    double dt = 0.;
    SphereGroup* spheregroup_ptr;

    // region
    int axis = 0;
    EigenVector3D position_center = EigenVector3D::Zero();
    double radius = 0.;
    double height = 0.;
    double radius_ratio = 0.;

    // timesteps
    int ts_start = 0;
    int ts_end = 0;
    int ts_step = 0;

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    DeleteSphereCylinder() {}

    // constructor
    DeleteSphereCylinder(
        SphereGroup &spheregroup_in, int axis_in, EigenVector3D position_center_in, double radius_in, double height_in,
        int ts_start_in = 0, int ts_end_in = -1, int ts_step_in = -1, double radius_ratio_in = 0
    )
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;

        // regions
        axis = axis_in;
        position_center = position_center_in;
        radius = radius_in;
        height = height_in;

        // timesteps
        ts_start = ts_start_in;
        ts_end = ts_end_in;
        ts_step = ts_step_in;

    }

    private:

};

void DeleteSphereCylinder::update(int ts)
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

        // check if sphere is outside region
        double radius_sphere = it->radius;
        bool is_delete = false;
        switch (axis)
        {
        case 0: // x-axis
        {
            double height_min = position_center(0) - 0.5*height;
            double height_max = position_center(0) + 0.5*height;
            double radial_distance = std::sqrt((pos(1) - position_center(1)) * (pos(1) - position_center(1)) + (pos(2) - position_center(2)) * (pos(2) - position_center(2)));
            is_delete = (pos(0) >= height_min - radius_ratio*radius_sphere && pos(0) <= height_max + radius_ratio*radius_sphere) && (radial_distance <= radius + radius_ratio*radius_sphere);
            break;
        }
        case 1: // y-axis
        {
            double height_min = position_center(1) - 0.5*height;
            double height_max = position_center(1) + 0.5*height;
            double radial_distance = std::sqrt((pos(0) - position_center(0)) * (pos(0) - position_center(0)) + (pos(2) - position_center(2)) * (pos(2) - position_center(2)));
            is_delete = (pos(1) >= height_min - radius_ratio*radius_sphere && pos(1) <= height_max + radius_ratio*radius_sphere) && (radial_distance <= radius + radius_ratio*radius_sphere);
            break;
        }
        case 2: // z-axis
        {
            double height_min = position_center(2) - 0.5*height;
            double height_max = position_center(2) + 0.5*height;
            double radial_distance = std::sqrt((pos(0) - position_center(0)) * (pos(0) - position_center(0)) + (pos(1) - position_center(1)) * (pos(1) - position_center(1)));
            is_delete = (pos(2) >= height_min - radius_ratio*radius_sphere && pos(2) <= height_max + radius_ratio*radius_sphere) && (radial_distance <= radius + radius_ratio*radius_sphere);
            break;
        }
        }

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
