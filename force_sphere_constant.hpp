#ifndef FORCE_SPHERE_CONSTANT
#define FORCE_SPHERE_CONSTANT
#include "container_typedef.hpp"
#include "forcemoment_base.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ForceSphereConstant : public ForceMomentBase
{
    /*

    Applies a constant force on sphere objects.

    Variables
    =========
    spheregroup_in : SphereGroup
        Spheres where force is applied.
    force_in : Eigen::Vector3d
        Constant force to be applied.

    Functions
    =========
    get_group_ptr_vec : vector<BaseGroup*>
        Returns pointers to group objects affected by this object.
    update : void
        Updates this object.

    */

    public:

    // sphere group
    SphereGroup* spheregroup_ptr;

    // constant force
    EigenVector3D force;

    // functions
    std::vector<BaseGroup*> get_group_ptr_vec() {return {spheregroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    ForceSphereConstant() {}

    // constructor
    ForceSphereConstant(SphereGroup &spheregroup_in, EigenVector3D force_in)
    {

        // store variables
        spheregroup_ptr = &spheregroup_in;
        force = force_in;

    }

    private:

};

void ForceSphereConstant::update(int ts, double dt)
{
    /*

    Updates this object.

    Arguments
    =========
    ts : int
        Timestep number.
    dt : double
        Duration of timestep.

    Returns
    =======
    (none)

    */

    // add force to particles
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        sphere.force += force;
    }

}

}

#endif
