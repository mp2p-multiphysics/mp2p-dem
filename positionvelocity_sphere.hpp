#ifndef POSITIONVELOCITY_SPHERE
#define POSITIONVELOCITY_SPHERE
#include <fstream>
#include <sstream>
#include "container_typedef.hpp"
#include "group_sphere.hpp"
#include "parameter_unary.hpp"
#include "positionvelocity_base.hpp"

namespace DEM
{

class PositionVelocitySphere : public PositionVelocityBase
{

    public:

    // sphere group
    SphereGroup* spheregroup_ptr;

    // parameters
    ParameterUnary* density_ptr;

    // functions
    std::vector<BaseGroup*> get_group_ptr_vec() {return {spheregroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    PositionVelocitySphere() {}

    // constructor
    PositionVelocitySphere(SphereGroup &spheregroup_in, ParameterUnary &density_in)
    {

        // store variables
        spheregroup_ptr = &spheregroup_in;
        density_ptr = &density_in;

    }

    private:

};

void PositionVelocitySphere::update(int ts, double dt)
{

    // iterate through each sphere
    for (int indx_i = 0; indx_i < spheregroup_ptr->num_sphere; indx_i++)
    {

        // get spheres
        Sphere sphere_prev = spheregroup_ptr->sphere_previous_ts_vec[indx_i];
        Sphere sphere = spheregroup_ptr->sphere_vec[indx_i];
        Sphere sphere_next = sphere; // copy to modify

        // get properties
        int mid = sphere.mid;
        double radius = sphere.radius;
        double density = density_ptr->get_value(mid);

        // calculate masses
        double mass = (4./3.)*M_PI*density*radius*radius*radius;
        double moi = 0.4*mass*radius*radius;

        // get new position
        sphere_next.position = 2*sphere.position - sphere_prev.position + sphere.force*dt*dt/mass;
        sphere_next.angularposition = 2*sphere.angularposition - sphere_prev.angularposition + sphere.moment*dt*dt/moi;

        // get new velocity
        sphere_next.velocity = 0.5*(sphere_next.position - sphere_prev.position)/dt;
        sphere_next.angularvelocity = 0.5*(sphere_next.angularposition - sphere_prev.angularposition)/dt;

        // reset forces and moments
        sphere_next.force = {0., 0., 0.,};
        sphere_next.moment = {0., 0., 0.,};

        // store spheres
        spheregroup_ptr->sphere_previous_ts_vec[indx_i] = sphere;
        spheregroup_ptr->sphere_vec[indx_i] = sphere_next;

    }

}

}

#endif
