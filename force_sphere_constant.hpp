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

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // sphere group
    SphereGroup* spheregroup_ptr;

    // constant force
    EigenVector3D force = EigenVector3D::Zero();

    // functions
    void initialize(double dt_in) {};
    void update(int ts);

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

void ForceSphereConstant::update(int ts)
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

    // add force to particles
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        sphere.force += force;
    }

}

}

#endif
