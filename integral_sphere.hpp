#ifndef INTEGRAL_SPHERE
#define INTEGRAL_SPHERE
#include "container_typedef.hpp"
#include "group_sphere.hpp"
#include "integral_base.hpp"
#include "parameter_unary.hpp"

namespace DEM
{

class IntegralSphere : public IntegralBase
{
    /*

    Updates the position and velocities of spheres via velocity Verlet integration.

    Variables
    =========
    spheregroup_in : SphereGroup
        Spheres whose positions and velocities are updated.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // sphere group
    double dt = 0.;
    SphereGroup* spheregroup_ptr;

    // parameters
    ParameterUnary* density_ptr;

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    IntegralSphere() {}

    // constructor
    IntegralSphere(SphereGroup &spheregroup_in, ParameterUnary &density_in)
    {

        // store variables
        spheregroup_ptr = &spheregroup_in;
        density_ptr = &density_in;

    }

    private:

};

void IntegralSphere::update(int ts)
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
    assert(spheregroup_ptr != nullptr);
    // iterate through each sphere
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {

        // get properties
        int mid = sphere.mid;
        double radius = sphere.radius;
        double density = density_ptr->get_value(mid);

        // calculate masses
        double mass = (4./3.)*M_PI*density*radius*radius*radius;
        double moi = 0.4*mass*radius*radius;

        // store current position
        EigenVector3D pos = sphere.position;
        EigenVector3D angpos = sphere.angularposition;

        // get new position
        sphere.position = 2*pos - sphere.previous_position + sphere.force*dt*dt/mass;
        sphere.angularposition = 2*angpos - sphere.previous_angularposition + sphere.moment*dt*dt/moi;

        // get new velocity
        sphere.velocity = 0.5*(sphere.position - sphere.previous_position)/dt;
        sphere.angularvelocity = 0.5*(sphere.angularposition - sphere.previous_angularposition)/dt;

        // current position -> previous position
        sphere.previous_position = pos;
        sphere.previous_angularposition = angpos;

        // reset forces and moments
        sphere.force = {0., 0., 0.,};
        sphere.moment = {0., 0., 0.,};

    }

}

}

#endif
