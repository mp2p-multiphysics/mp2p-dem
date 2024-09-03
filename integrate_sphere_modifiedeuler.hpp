#ifndef INTEGRATE_SPHERE_MODIFIEDEULER
#define INTEGRATE_SPHERE_MODIFIEDEULER
#include <vector>
#include "container_sphere.hpp"
#include "container_typedef.hpp"

class IntegrateSphereModifiedEuler
{

    public:

    // variables
    VectorDouble radius_vec;
    VectorDouble density_vec;
    double dt;

    // functions
    void integrate_positionvelocity
    (
        SpherePositionVelocityStruct &sphere_pvs,  
        SphereForceMomentStruct sphere_fms
    );

    // default constructor
    IntegrateSphereModifiedEuler()
    {

    }

    // constructor
    IntegrateSphereModifiedEuler(VectorDouble radius_vec_in, VectorDouble density_vec_in, double dt_in)
    {
        radius_vec = radius_vec_in;
        density_vec = density_vec_in;
        dt = dt_in;
    }

};

void IntegrateSphereModifiedEuler::integrate_positionvelocity
(
    SpherePositionVelocityStruct &sphere_pvs,   
    SphereForceMomentStruct sphere_fms
)
{

    // update positions and velocities
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++)
    {
        
        // get particle type
        int type_i = sphere_pvs.type_vec[indx_i];

        // calculate particle mass and moment of inertia
        double mass_i = density_vec[type_i]*(4.*M_PI/3.)*radius_vec[type_i]*radius_vec[type_i]*radius_vec[type_i];
        double mmoi_i = 0.4*mass_i*radius_vec[type_i]*radius_vec[type_i];

        // calculate number of contacts for averaging
        int num_contact = sphere_fms.num_contact_vec[indx_i];
        double inv_num_contact = 0.;
        if (num_contact != 0)
        {
            inv_num_contact = 1./(double) num_contact;
        }

        // calculate net forces and moments
        double force_net_x = sphere_fms.force_sum_x_vec[indx_i] + inv_num_contact*sphere_fms.force_average_x_vec[indx_i];
        double force_net_y = sphere_fms.force_sum_y_vec[indx_i] + inv_num_contact*sphere_fms.force_average_y_vec[indx_i];
        double force_net_z = sphere_fms.force_sum_z_vec[indx_i] + inv_num_contact*sphere_fms.force_average_z_vec[indx_i];
        double moment_net_x = sphere_fms.moment_sum_x_vec[indx_i] + inv_num_contact*sphere_fms.moment_average_x_vec[indx_i];
        double moment_net_y = sphere_fms.moment_sum_y_vec[indx_i] + inv_num_contact*sphere_fms.moment_average_y_vec[indx_i];
        double moment_net_z = sphere_fms.moment_sum_z_vec[indx_i] + inv_num_contact*sphere_fms.moment_average_z_vec[indx_i];

        // update velocities using forces
        sphere_pvs.velocity_x_vec[indx_i] += force_net_x*dt/mass_i;
        sphere_pvs.velocity_y_vec[indx_i] += force_net_y*dt/mass_i;
        sphere_pvs.velocity_z_vec[indx_i] += force_net_z*dt/mass_i;

        // update positions using updated velocities
        sphere_pvs.position_x_vec[indx_i] += sphere_pvs.velocity_x_vec[indx_i]*dt;
        sphere_pvs.position_y_vec[indx_i] += sphere_pvs.velocity_y_vec[indx_i]*dt;
        sphere_pvs.position_z_vec[indx_i] += sphere_pvs.velocity_z_vec[indx_i]*dt;

        // update angular velocities using forces
        sphere_pvs.angularvelocity_x_vec[indx_i] += moment_net_x*dt/mmoi_i;
        sphere_pvs.angularvelocity_y_vec[indx_i] += moment_net_y*dt/mmoi_i;
        sphere_pvs.angularvelocity_z_vec[indx_i] += moment_net_z*dt/mmoi_i;

        // update angular positions using updated angular velocities
        sphere_pvs.angularposition_x_vec[indx_i] += sphere_pvs.angularvelocity_x_vec[indx_i]*dt;
        sphere_pvs.angularposition_y_vec[indx_i] += sphere_pvs.angularvelocity_y_vec[indx_i]*dt;
        sphere_pvs.angularposition_z_vec[indx_i] += sphere_pvs.angularvelocity_z_vec[indx_i]*dt;

    }

}

#endif
