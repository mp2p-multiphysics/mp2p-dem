#ifndef INSERT_SPHERE_BOX_INSIDE_NUMBER
#define INSERT_SPHERE_BOX_INSIDE_NUMBER
#include <fstream>
#include <random>
#include <sstream>
#include <vector>
#include "container_sphere.hpp"

class InsertSphereBoxInsideNumber
{
    /*

    Inserts spheres inside a box-shaped region.

    Variables
    =========
    radius_vec_in : VectorDouble
        vector with the radius of each type of sphere.
    number_fraction_vec_in : VectorDouble
        vector with the fraction (by number) of each type of sphere to be generated.
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
    num_insert_sphere_in : int
        Total number of spheres to be inserted.
    num_insert_instance_in : int
        Number of times that spheres will be inserted.
    num_max_insert_attempt_in : int
        Maximum number of attempts that will be done to insert particles in the box.
    seed_in : int
        Seed for random number generation.

    Functions
    =========
    insert_sphere : void
        Inserts spheres into the simulation.

    Notes
    =====
    This class inserts spheres that are fully inside the box.

    */

    public:

    // particle radius and number fraction
    VectorDouble radius_vec;
    VectorDouble number_fraction_vec;

    // geometry
    double box_min_x = 0.;
    double box_min_y = 0.;
    double box_min_z = 0.;
    double box_max_x = 0.;
    double box_max_y = 0.;
    double box_max_z = 0.;

    // insertion settings
    int num_insert_sphere = 0;
    int num_insert_instance = 0;
    int num_max_insert_attempt = 0;
    int seed = 0;

    // functions
    void insert_sphere(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    InsertSphereBoxInsideNumber()
    {

    }

    // constructor
    InsertSphereBoxInsideNumber
    (
        VectorDouble radius_vec_in, VectorDouble number_fraction_vec_in,
        double box_min_x_in, double box_min_y_in, double box_min_z_in,
        double box_max_x_in, double box_max_y_in, double box_max_z_in,
        int num_insert_sphere_in, int num_insert_instance_in,
        int num_max_insert_attempt_in = 10000, int seed_in = 42
    )
    {
        
        // particle radius and number fraction
        radius_vec = radius_vec_in;
        number_fraction_vec = number_fraction_vec_in;

        // geometry
        box_min_x = box_min_x_in;
        box_min_y = box_min_y_in;
        box_min_z = box_min_z_in;
        box_max_x = box_max_x_in;
        box_max_y = box_max_y_in;
        box_max_z = box_max_z_in;

        // insertion settings
        num_insert_sphere = num_insert_sphere_in;
        num_insert_instance = num_insert_instance_in;
        num_max_insert_attempt = num_max_insert_attempt_in;

        // insertion statistics
        num_insert_sphere_remaining = num_insert_sphere;
        num_insert_instance_remaining = num_insert_instance;
        seed = seed_in;

    }

    private:

    // insertion statistics
    int num_insert_sphere_remaining = 0;
    int num_insert_instance_remaining = 0;
    
    // random number generator
    bool is_first_insertion = true;
    std::mt19937 rng;

};

void InsertSphereBoxInsideNumber::insert_sphere(SpherePositionVelocityStruct &sphere_pvs)
{
    /*

    Inserts spheres into the simulation.

    Arguments
    =========
    sphere_pvs : SpherePositionVelocityStruct
        struct with position and velocity of each sphere.

    Returns
    =======
    (none)

    */

    // seed random number generator if first instance
    if (is_first_insertion)
    {
        rng.seed(seed);
    }

    // determine number of particles to insert in this instance
    int num_insert_sphere_now = num_insert_sphere_remaining/num_insert_instance_remaining;

    // initialize vector of inserted particles
    int num_insert_particle_success = 0;
    VectorInt type_insert_vec;
    VectorDouble pos_x_insert_vec;
    VectorDouble pos_y_insert_vec;
    VectorDouble pos_z_insert_vec;

    // iterate through each particle to be inserted
    for (int indx_i = 0; indx_i < num_insert_sphere_now; indx_i++)
    {

        // initialize for deciding what particle type to generate
        double decide_type_i = double(rng())/double(rng.max() - rng.min());
        double number_fraction_cumulative = 0.;
        int type_i = 0;

        // decide what particle type to generate based on number fraction
        for (auto &number_fraction : number_fraction_vec)
        {

            // increment cumulative number fraction
            number_fraction_cumulative += number_fraction;

            // create particle type_i if decide_type_i < number_fraction_cumulative
            if (decide_type_i < number_fraction_cumulative)
            {
                break;
            }

            // increment particle type
            type_i++;

        }

        // get particle radius
        double radius_i = radius_vec[type_i];

        // attempt particle insertion
        for (int attempt = 0; attempt < num_max_insert_attempt; attempt++)
        {

            // generate random numbers from 0 and 1
            // this will be a dimensionless coordinate along the x, y, and z axes
            double rel_pos_x_i = double(rng())/double(rng.max() - rng.min());
            double rel_pos_y_i = double(rng())/double(rng.max() - rng.min());
            double rel_pos_z_i = double(rng())/double(rng.max() - rng.min());

            // calculate x, y, and z coordinate of inserted particle
            // generated particle is within bounding box
            double pos_x_i = (box_min_x + radius_i) + (box_max_x - box_min_x - 2*radius_i)*rel_pos_x_i;
            double pos_y_i = (box_min_y + radius_i) + (box_max_y - box_min_y - 2*radius_i)*rel_pos_y_i;
            double pos_z_i = (box_min_z + radius_i) + (box_max_z - box_min_z - 2*radius_i)*rel_pos_z_i;

            // initialize flag indicating successful particle insertion
            bool is_insert_success = true;

            // verify that new particle does not collide with previously inserted particles
            for (int indx_j = 0; indx_j < indx_i; indx_j++)
            {

                // get type of previously inserted particles
                int type_j = type_insert_vec[indx_j];

                // get coordintes of previously inserted particles
                double pos_x_j = pos_x_insert_vec[indx_j];
                double pos_y_j = pos_y_insert_vec[indx_j];
                double pos_z_j = pos_z_insert_vec[indx_j];

                // get radii of previously inserted particles
                double radius_j = radius_vec[type_j];

                // calculate displacement from center of i to j
                double delta_pos_x_ij = -pos_x_i + pos_x_j;
                double delta_pos_y_ij = -pos_y_i + pos_y_j;
                double delta_pos_z_ij = -pos_z_i + pos_z_j;

                // calculate distance from center of i to j
                double delta_pos_ij_mag = sqrt(delta_pos_x_ij*delta_pos_x_ij + delta_pos_y_ij*delta_pos_y_ij + delta_pos_z_ij*delta_pos_z_ij);

                // calculate normal overlap
                double overlap_normal_ij_val = -delta_pos_ij_mag + radius_i + radius_j;

                // redo particle insertion if overlap exists
                if (overlap_normal_ij_val > 0)
                {
                    is_insert_success = false;
                    break;
                }

            }

            // reattempt if insertion is unsuccessful
            if (!is_insert_success)
            {
                continue;
            }

            // add new particle into list of inserted particles
            type_insert_vec.push_back(type_i);
            pos_x_insert_vec.push_back(pos_x_i);
            pos_y_insert_vec.push_back(pos_y_i);
            pos_z_insert_vec.push_back(pos_z_i);
            num_insert_particle_success++;

            // stop if particle has been inserted
            break;

        }

        // stop if all particles are inserted
        if (num_insert_particle_success == num_insert_sphere_now)
        {
            break;
        }

    }

    // add particles to simulation
    for (int indx_i; indx_i < num_insert_particle_success; indx_i++)
    {
        sphere_pvs.num_particle++;
        sphere_pvs.id_vec.push_back(sphere_pvs.num_particle_historical_max + indx_i);
        sphere_pvs.type_vec.push_back(type_insert_vec[indx_i]);
        sphere_pvs.position_x_vec.push_back(pos_x_insert_vec[indx_i]);
        sphere_pvs.position_y_vec.push_back(pos_y_insert_vec[indx_i]);
        sphere_pvs.position_z_vec.push_back(pos_z_insert_vec[indx_i]);
        sphere_pvs.velocity_x_vec.push_back(0.);
        sphere_pvs.velocity_y_vec.push_back(0.);
        sphere_pvs.velocity_z_vec.push_back(0.);
        sphere_pvs.angularposition_x_vec.push_back(0.);
        sphere_pvs.angularposition_y_vec.push_back(0.);
        sphere_pvs.angularposition_z_vec.push_back(0.);
        sphere_pvs.angularvelocity_x_vec.push_back(0.);
        sphere_pvs.angularvelocity_y_vec.push_back(0.);
        sphere_pvs.angularvelocity_z_vec.push_back(0.);
    }
    
    // increment historical maximum particle count
    sphere_pvs.num_particle_historical_max += num_insert_particle_success;

}

#endif
