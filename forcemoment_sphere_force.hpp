#ifndef FORCEMOMENT_SPHERE_FORCE
#define FORCEMOMENT_SPHERE_FORCE
#include "container_typedef.hpp"
#include "forcemoment_base.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class ForceMomentSphereForce : public ForceMomentBase
{

    public:

    // sphere group
    SphereGroup* spheregroup_ptr;

    // constant force
    EigenVector3D force;

    // functions
    std::vector<BaseGroup*> get_group_ptr_vec() {return {spheregroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    ForceMomentSphereForce() {}

    // constructor
    ForceMomentSphereForce(SphereGroup &spheregroup_in, EigenVector3D force_in)
    {

        // store variables
        spheregroup_ptr = &spheregroup_in;
        force = force_in;

    }

    private:

};

void ForceMomentSphereForce::update(int ts, double dt)
{

    // add force to particles
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        sphere.force += force;
    }

}

}

#endif
