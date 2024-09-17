#ifndef CONTAINER_SPHERE
#define CONTAINER_SPHERE
#include <vector>
#include "container_typedef.hpp"

struct SpherePositionVelocityStruct
{
    /*
    
    Stores position and velocity of each sphere.

    Variables
    =========
    num_particle : int
        Number of spheres.
    num_particle_historical_max : int
        Highest recorded number of spheres in the simulation.
    id_vec : int
        ID of each sphere.
    type_vec : int
        Type of each sphere.
    position_x_vec : VectorDouble
        x-component of the position of each particle.
    position_y_vec : VectorDouble
        y-component of the position of each particle.
    position_z_vec : VectorDouble
        z-component of the position of each particle.
    velocity_x_vec : VectorDouble
        x-component of the velocity of each particle.
    velocity_y_vec : VectorDouble
        y-component of the velocity of each particle.
    velocity_z_vec : VectorDouble
        z-component of the velocity of each particle.
    angularposition_x_vec : VectorDouble
        x-component of the angular position of each particle.
    angularposition_y_vec : VectorDouble
        y-component of the angular position of each particle.
    angularposition_z_vec : VectorDouble
        z-component of the angular position of each particle.
    angularvelocity_x_vec : VectorDouble
        x-component of the angular velocity of each particle.
    angularvelocity_y_vec : VectorDouble
        y-component of the angular velocity of each particle.
    angularvelocity_z_vec : VectorDouble
        z-component of the angular velocity of each particle.

    */

    // number of particles
    int num_particle = 0;
    int num_particle_historical_max = 0;

    // id and type of particles
    VectorInt id_vec;
    VectorInt type_vec;

    // translational positions and velocities
    VectorDouble position_x_vec;
    VectorDouble position_y_vec;
    VectorDouble position_z_vec;
    VectorDouble velocity_x_vec;
    VectorDouble velocity_y_vec;
    VectorDouble velocity_z_vec;

    // rotational positions and velocities
    VectorDouble angularposition_x_vec;
    VectorDouble angularposition_y_vec;
    VectorDouble angularposition_z_vec;
    VectorDouble angularvelocity_x_vec;
    VectorDouble angularvelocity_y_vec;
    VectorDouble angularvelocity_z_vec;

};

struct SphereForceMomentStruct
{
    /*
    
    Stores forces and moments acting on each sphere.

    Variables
    =========
    num_particle : int
        Number of spheres.
    force_sum_x_vec : VectorDouble
        x-component of the total force (excluding those needing averaging) acting on each particle.
    force_sum_y_vec : VectorDouble
        y-component of the total force (excluding those needing averaging) acting on each particle.
    force_sum_z_vec : VectorDouble
        z-component of the total force (excluding those needing averaging) acting on each particle.
    moment_sum_x_vec : VectorDouble
        x-component of the total moment (excluding those needing averaging) acting on each particle.
    moment_sum_y_vec : VectorDouble
        y-component of the total moment (excluding those needing averaging) acting on each particle.
    moment_sum_z_vec : VectorDouble
        z-component of the total moment (excluding those needing averaging) acting on each particle.
    num_contact_vec : VectorInt
        Number of collisions that need to be averaged.
    force_average_x_vec : VectorDouble
        x-component of the total force (excluding those already summed) acting on each particle.
    force_average_y_vec : VectorDouble
        y-component of the total force (excluding those already summed) acting on each particle.
    force_average_z_vec : VectorDouble
        z-component of the total force (excluding those already summed) acting on each particle.
    moment_average_x_vec : VectorDouble
        x-component of the total moment (excluding those already summed) acting on each particle.
    moment_average_y_vec : VectorDouble
        y-component of the total moment (excluding those already summed) acting on each particle.
    moment_average_z_vec : VectorDouble
        z-component of the total moment (excluding those already summed) acting on each particle.

    Notes
    =====
    Forces and moments already summed (e.g., force_sum) and needing averaging (e.g., force_average) are stored separately.
    This is because if a particle hits an edge or vertex, then the net force is the average of the forces exerted by the surrounding faces.
    As an example, the net force along the x-direction is force_sum_x + force_average_x/num_contact.

    */

    // number of particles
    int num_particle = 0;

    // total force and moment (excluding those needing averaging)
    VectorDouble force_sum_x_vec;
    VectorDouble force_sum_y_vec;
    VectorDouble force_sum_z_vec;
    VectorDouble moment_sum_x_vec;
    VectorDouble moment_sum_y_vec;
    VectorDouble moment_sum_z_vec;

    // total force and moment (excluding those already summed)
    VectorInt num_contact_vec;
    VectorDouble force_average_x_vec;
    VectorDouble force_average_y_vec;
    VectorDouble force_average_z_vec;
    VectorDouble moment_average_x_vec;
    VectorDouble moment_average_y_vec;
    VectorDouble moment_average_z_vec;

};

SphereForceMomentStruct sphere_fms_fill(int num_particle)
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
