#ifndef CONTAINER_SPHERE
#define CONTAINER_SPHERE
#include <vector>
#include "container_typedef.hpp"

struct SpherePositionVelocityStruct
{
   
    int num_particle = 0;
    int num_particle_historical_max = 0;

    VectorInt id_vec;
    VectorInt type_vec;

    VectorDouble position_x_vec;
    VectorDouble position_y_vec;
    VectorDouble position_z_vec;
    VectorDouble velocity_x_vec;
    VectorDouble velocity_y_vec;
    VectorDouble velocity_z_vec;

    VectorDouble angularposition_x_vec;
    VectorDouble angularposition_y_vec;
    VectorDouble angularposition_z_vec;
    VectorDouble angularvelocity_x_vec;
    VectorDouble angularvelocity_y_vec;
    VectorDouble angularvelocity_z_vec;

};

struct SphereForceMomentStruct
{

    int num_particle = 0;

    VectorDouble force_sum_x_vec;
    VectorDouble force_sum_y_vec;
    VectorDouble force_sum_z_vec;
    VectorDouble moment_sum_x_vec;
    VectorDouble moment_sum_y_vec;
    VectorDouble moment_sum_z_vec;

    VectorInt num_contact_vec;
    VectorDouble force_average_x_vec;
    VectorDouble force_average_y_vec;
    VectorDouble force_average_z_vec;
    VectorDouble moment_average_x_vec;
    VectorDouble moment_average_y_vec;
    VectorDouble moment_average_z_vec;

};

#endif
