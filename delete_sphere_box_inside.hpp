#ifndef DELETE_SPHERE_BOX_INSIDE
#define DELETE_SPHERE_BOX_INSIDE
#include <vector>
#include "container_sphere.hpp"

class DeleteSphereBoxInside
{
    /*

    Deletes spheres inside a box-shaped region.

    Variables
    =========
    radius_vec_in : VectorDouble
        vector with the radius of each type of sphere.
    box_min_x_in : double
        Lower x-coordinate of the box.
    box_min_y_in : double
        Lower y-coordinate of the box.
    box_min_z_in : double
        Lower z-coordinate of the box.
    box_max_x_in : double
        Upper x-coordinate of the box.
    box_max_y_in : double
        Upper y-coordinate of the box.
    box_max_z_in : double
        Upper z-coordinate of the box.

    Functions
    =========
    delete_sphere : void
        Deletes spheres from the simulation.

    Notes
    =====
    This class deletes spheres that are partially or fully inside the box.
    Deletion occurs even if the center of the sphere is not inside the box.

    */

    public:

    // particle radius
    VectorDouble radius_vec;

    // geometry
    double box_min_x = 0.;
    double box_min_y = 0.;
    double box_min_z = 0.;
    double box_max_x = 0.;
    double box_max_y = 0.;
    double box_max_z = 0.;

    // functions
    void delete_sphere(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    DeleteSphereBoxInside()
    {

    }

    // constructor
    DeleteSphereBoxInside
    (
        VectorDouble radius_vec_in,
        double box_min_x_in, double box_min_y_in, double box_min_z_in,
        double box_max_x_in, double box_max_y_in, double box_max_z_in
    )
    {
        
        // particle radius
        radius_vec = radius_vec_in;

        // geometry
        box_min_x = box_min_x_in;
        box_min_y = box_min_y_in;
        box_min_z = box_min_z_in;
        box_max_x = box_max_x_in;
        box_max_y = box_max_y_in;
        box_max_z = box_max_z_in;

    }
    
};

void DeleteSphereBoxInside::delete_sphere(SpherePositionVelocityStruct &sphere_pvs)
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

        // skip if outside bounding box
        if (
            pos_x_i + radius_i <= box_min_x || pos_x_i - radius_i >= box_max_x ||
            pos_y_i + radius_i <= box_min_y || pos_y_i - radius_i >= box_max_y ||
            pos_z_i + radius_i <= box_min_z || pos_z_i - radius_i >= box_max_z
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
