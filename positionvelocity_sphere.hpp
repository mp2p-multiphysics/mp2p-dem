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
    GroupSphere* spheregroup_ptr;

    // parameters
    ParameterUnary* density_ptr;

    // functions
    std::vector<GroupBase*> get_group_ptr_vec() {return {spheregroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    PositionVelocitySphere() {}

    // constructor
    PositionVelocitySphere(GroupSphere &spheregroup_in, ParameterUnary &density_in)
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
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        
        // get properties
        int mid = sphere.mid;
        double radius = sphere.radius;
        double density = density_ptr->get_value(mid);

        // calculate masses
        double mass = (4./3.)*M_PI*density*radius*radius*radius;
        double moi = 0.4*mass*radius*radius;
        
        // calculate linear position and velocity
        sphere.velocity += sphere.force*dt/mass;
        sphere.position += sphere.velocity*dt;

        // calculate rotational position and velocity
        sphere.angularvelocity += sphere.moment*dt/moi;
        sphere.angularposition += sphere.angularvelocity*dt;

    }

}

}

#endif
