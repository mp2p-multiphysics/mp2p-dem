#ifndef FORCE_SPHERE_CONSTANT
#define FORCE_SPHERE_CONSTANT
#include <vector>
#include "container_sphere.hpp"

class ForceSphereConstant
{

    public:

    // variables
    double force_x;
    double force_y;
    double force_z;

    // functions
    void add_force_moment(SphereForceMomentStruct &sphere_fms);

    // default constructor
    ForceSphereConstant()
    {

    }

    // constructor
    ForceSphereConstant(double force_x_in, double force_y_in, double force_z_in)
    {
        force_x = force_x_in;
        force_y = force_y_in;
        force_z = force_z_in;
    }

};

void ForceSphereConstant::add_force_moment(SphereForceMomentStruct &sphere_fms)
{

    // add forces
    for (int indx_i = 0; indx_i < sphere_fms.num_particle; indx_i++)
    {
        sphere_fms.force_sum_x_vec[indx_i] += force_x;
        sphere_fms.force_sum_y_vec[indx_i] += force_y;
        sphere_fms.force_sum_z_vec[indx_i] += force_z;
    }

}

#endif
