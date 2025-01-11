#ifndef PHYSICS_SPHERE_FORCE
#define PHYSICS_SPHERE_FORCE
#include "container_typedef.hpp"
#include "particle_sphere.hpp"
#include "physics_base.hpp"

namespace DEM
{

class PhysicsSphereForce : public PhysicsBase
{

    public:
    
    // particle object to insert to
    ParticleSphere* sphere_ptr;

    // force to apply
    EigenVector3D force_constant;

    // functions used in timestepping
    void compute_force(int ts);
    void compute_position_velocity(int ts) {};
    void compute_insert_delete(int ts) {};

    // default constructor
    PhysicsSphereForce() {}

    // constructor
    PhysicsSphereForce(ParticleSphere &sphere_in, EigenVector3D force_in)
    {

        // store inputs
        sphere_ptr = &sphere_in;
        force_constant = force_in;

    }

    private:

};

void PhysicsSphereForce::compute_force(int ts)
{

    // add force
    for (int indx_i = 0; indx_i < sphere_ptr->num_element; indx_i++)
    {
        sphere_ptr->force_vec[indx_i] += force_constant;
    }

}

}

#endif
