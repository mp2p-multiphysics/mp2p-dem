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

    */

    public:

    // sphere group
    double dt = 0.;
    SphereGroup* spheregroup_ptr;

    // region
    EigenVector3D position_min;
    EigenVector3D position_max;

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
    for (int i = 0; i < spheregroup_ptr->sphere_vec.size(); i++)
    {

        // get sphere position
        EigenVector3D pos = spheregroup_ptr->sphere_vec[i].position;

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
            spheregroup_ptr->gid_to_tid_map.erase(spheregroup_ptr->sphere_vec[i].gid);
            spheregroup_ptr->tid_to_gid_vec.erase(spheregroup_ptr->tid_to_gid_vec.begin() + i);
            spheregroup_ptr->sphere_vec.erase(spheregroup_ptr->sphere_vec.begin() + i);
            i--; // wind back index
        }

    }
    
}

}

#endif
