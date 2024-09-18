#ifndef INITIALIZE_SPHERE_FORCEMOMENT_ZERO
#define INITIALIZE_SPHERE_FORCEMOMENT_ZERO
#include "container_sphere.hpp"
#include "container_typedef.hpp"

SphereForceMomentStruct initialize_sphere_forcemoment_zero(int num_particle)
{
    /*
    
    Creates a SphereForceMomentStruct with zero forces and moments.

    Arguments
    =========
    num_particle : int
        Number of particles in the simulation.

    Returns
    =======
    sphere_fms : SphereForceMomentStruct
        SphereForceMomentStruct with zero forces and moments.

    */

    // initialize
    SphereForceMomentStruct sphere_fms;
    VectorDouble zeros_vecd(num_particle, 0.);
    VectorInt zeros_veci(num_particle, 0);

    // fill struct with zeros
    sphere_fms.num_particle = num_particle;
    sphere_fms.force_sum_x_vec = zeros_vecd;
    sphere_fms.force_sum_y_vec = zeros_vecd;
    sphere_fms.force_sum_z_vec = zeros_vecd;
    sphere_fms.moment_sum_x_vec = zeros_vecd;
    sphere_fms.moment_sum_y_vec = zeros_vecd;
    sphere_fms.moment_sum_z_vec = zeros_vecd;
    sphere_fms.force_average_x_vec = zeros_vecd;
    sphere_fms.force_average_y_vec = zeros_vecd;
    sphere_fms.force_average_z_vec = zeros_vecd;
    sphere_fms.moment_average_x_vec = zeros_vecd;
    sphere_fms.moment_average_y_vec = zeros_vecd;
    sphere_fms.moment_average_z_vec = zeros_vecd;
    sphere_fms.num_contact_vec = zeros_veci;

    return sphere_fms;

}

#endif
