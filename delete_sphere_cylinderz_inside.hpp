#ifndef DELETE_SPHERE_CYLINDERZ_INSIDE
#define DELETE_SPHERE_CYLINDERZ_INSIDE
#include <vector>
#include "container_sphere.hpp"

class DeleteSphereCylinderzInside
{
    /*

    Deletes spheres inside a cylinder with the y-axis as the axis of symmetry.

    Variables
    =========
    radius_vec_in : VectorDouble
        vector with the radius of each type of sphere.
    cylinder_center_x_in : double
        x-coordinate of the axis of symmetry.
    cylinder_center_y_in : double
        y-coordinate of the axis of symmetry.
    cylinder_min_z_in : double
        Lower z-coordinate of the cylinder.
    cylinder_max_z_in : double
        Upper z-coordinate of the cylinder.
    cylinder_radius_in : double
        Radius of the cylinder.

    Functions
    =========
    delete_sphere : void
        Deletes spheres from the simulation.

    Notes
    =====
    This class deletes spheres that are partially or fully inside the cylinder.
    Deletion occurs even if the center of the sphere is not inside the cylinder.

    */

    public:

    // particle radius
    VectorDouble radius_vec;

    // geometry
    double cylinder_center_x = 0.;
    double cylinder_center_y = 0.;
    double cylinder_min_z = 0.;
    double cylinder_max_z = 0.;
    double cylinder_radius = 0.;

    // functions
    void delete_sphere(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    DeleteSphereCylinderzInside()
    {

    }

    // constructor
    DeleteSphereCylinderzInside
    (
        VectorDouble radius_vec_in,
        double cylinder_center_x_in, double cylinder_center_y_in,
        double cylinder_min_z_in, double cylinder_max_z_in,
        double cylinder_radius_in
    )
    {
        
        // particle radius
        radius_vec = radius_vec_in;

        // geometry
        cylinder_center_x = cylinder_center_x_in;
        cylinder_center_y = cylinder_center_y_in;
        cylinder_min_z = cylinder_min_z_in;
        cylinder_max_z = cylinder_max_z_in;
        cylinder_radius = cylinder_radius_in;

    }
    
};

void DeleteSphereCylinderzInside::delete_sphere(SpherePositionVelocityStruct &sphere_pvs)
{
    /*

    Deletes spheres from the simulation.

    Arguments
    =========
    sphere_pvs : SpherePositionVelocityStruct
        struct with position and velocity of each sphere.

    Returns
    =======
    (none)

    */
    
    // delete particles outside of simulation box
    for (int iter_i = 0; iter_i < sphere_pvs.num_particle; iter_i++)
    {

        // get coordinates of particles
        double pos_x_i = sphere_pvs.position_x_vec[iter_i];
        double pos_y_i = sphere_pvs.position_y_vec[iter_i];
        double pos_z_i = sphere_pvs.position_z_vec[iter_i];

        // get radius of particles
        int type_i = sphere_pvs.type_vec[iter_i];
        double radius_i = radius_vec[type_i];

        // calculate distance of particle from axis
        double delta_pos_center_axis_x = pos_x_i - cylinder_center_x;
        double delta_pos_center_axis_y = pos_y_i - cylinder_center_y;        
        double dist_center_axis = sqrt(delta_pos_center_axis_x*delta_pos_center_axis_x + delta_pos_center_axis_y*delta_pos_center_axis_y);

        // skip if outside cylinder
        if (
            dist_center_axis - radius_i >= cylinder_radius ||
            pos_z_i + radius_i <= cylinder_min_z ||
            pos_z_i - radius_i >= cylinder_max_z
        )
        {
            continue;
        }

        // delete particle if outside of cylinder
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
