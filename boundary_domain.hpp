#ifndef BOUNDARY_DOMAIN
#define BOUNDARY_DOMAIN
#include <vector>
#include "container_sphere.hpp"

class BoundaryDomain
{

    public:

    // lower point
    double boundary_min_x = 0.;
    double boundary_min_y = 0.;
    double boundary_min_z = 0.;

    // upper point
    double boundary_max_x = 0.;
    double boundary_max_y = 0.;
    double boundary_max_z = 0.;

    // functions
    void remove_outofbounds_sphere(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    BoundaryDomain()
    {

    }

    // constructor
    BoundaryDomain(double boundary_min_x_in, double boundary_min_y_in, double boundary_min_z_in, double boundary_max_x_in, double boundary_max_y_in, double boundary_max_z_in)
    {
        
        // coordinates of lower point
        boundary_min_x = boundary_min_x_in;
        boundary_min_y = boundary_min_y_in;
        boundary_min_z = boundary_min_z_in;

        // coordinates of upper point
        boundary_max_x = boundary_max_x_in;
        boundary_max_y = boundary_max_y_in;
        boundary_max_z = boundary_max_z_in;        

    }
    
};

void BoundaryDomain::remove_outofbounds_sphere(SpherePositionVelocityStruct &sphere_pvs)
{

    // delete particles outside of simulation box
    for (int iter_i = 0; iter_i < sphere_pvs.num_particle; iter_i++)
    {

        // get coordinates of particles
        double pos_x_i = sphere_pvs.position_x_vec[iter_i];
        double pos_y_i = sphere_pvs.position_y_vec[iter_i];
        double pos_z_i = sphere_pvs.position_z_vec[iter_i];

        // skip if within bounding box
        if (
            pos_x_i >= boundary_min_x && pos_x_i <= boundary_max_x &&
            pos_y_i >= boundary_min_y && pos_y_i <= boundary_max_y &&
            pos_z_i >= boundary_min_z && pos_z_i <= boundary_max_z
        )
        {
            continue;
        }

        // delete particle if outside of bounding box
        sphere_pvs.id_vec.erase(sphere_pvs.id_vec.begin() + iter_i);
        sphere_pvs.type_vec.erase(sphere_pvs.type_vec.begin() + iter_i);
        sphere_pvs.position_x_vec.erase(sphere_pvs.position_x_vec.begin() + iter_i);
        sphere_pvs.position_y_vec.erase(sphere_pvs.position_y_vec.begin() + iter_i);
        sphere_pvs.position_z_vec.erase(sphere_pvs.position_z_vec.begin() + iter_i);
        sphere_pvs.velocity_x_vec.erase(sphere_pvs.velocity_x_vec.begin() + iter_i);
        sphere_pvs.velocity_y_vec.erase(sphere_pvs.velocity_y_vec.begin() + iter_i);
        sphere_pvs.velocity_z_vec.erase(sphere_pvs.velocity_z_vec.begin() + iter_i);
        sphere_pvs.angularposition_x_vec.erase(sphere_pvs.angularposition_x_vec.begin() + iter_i);
        sphere_pvs.angularposition_y_vec.erase(sphere_pvs.angularposition_y_vec.begin() + iter_i);
        sphere_pvs.angularposition_z_vec.erase(sphere_pvs.angularposition_z_vec.begin() + iter_i);
        sphere_pvs.angularvelocity_x_vec.erase(sphere_pvs.angularvelocity_x_vec.begin() + iter_i);
        sphere_pvs.angularvelocity_y_vec.erase(sphere_pvs.angularvelocity_y_vec.begin() + iter_i);
        sphere_pvs.angularvelocity_z_vec.erase(sphere_pvs.angularvelocity_z_vec.begin() + iter_i);

        // decrement iteration count and number of particles
        sphere_pvs.num_particle += -1;
        iter_i += -1;

    }

}

#endif
