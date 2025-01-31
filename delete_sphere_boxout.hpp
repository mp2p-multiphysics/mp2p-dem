#ifndef DELETE_SPHERE_BOXOUT
#define DELETE_SPHERE_BOXOUT
#include "insertdelete_base.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class DeleteSphereBoxOut : public InsertDeleteBase
{
    /*

    Deletes spheres outside a hexahedral region at specified times.

    Variables
    =========
    spheregroup_in : SphereGroup
        Sphere group to be deleted from.
    position_min_in : Eigen::Vector3d
        Minimum corner of the hexahedral region.
    position_max_in : Eigen::Vector3d
        Maximum corner of the hexahedral region.

    Notes
    =====
    This class only deletes spheres whose centers are outside the region.

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

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    DeleteSphereBoxOut() {}

    // constructor
    DeleteSphereBoxOut(SphereGroup &spheregroup_in, EigenVector3D position_min_in, EigenVector3D position_max_in)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;

        // regions
        position_min = position_min_in;
        position_max = position_max_in;

    }

    private:

};

void DeleteSphereBoxOut::update(int ts)
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

    // iterate through each sphere
    for (auto it = spheregroup_ptr->sphere_vec.begin(); it != spheregroup_ptr->sphere_vec.end();)
    {
        
        // get sphere position
        EigenVector3D pos = it->position;

        // check if sphere is outside region
        bool is_delete = (
            pos(0) < position_min(0) || pos(0) > position_max(0) ||  // x
            pos(1) < position_min(1) || pos(1) > position_max(1) ||  // y
            pos(2) < position_min(2) || pos(2) > position_max(2)     // z
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
