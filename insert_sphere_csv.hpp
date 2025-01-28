#ifndef INSERT_SPHERE_CSV
#define INSERT_SPHERE_CSV
#include <fstream>
#include <sstream>
#include "insertdelete_base.hpp"
#include "group_sphere.hpp"

namespace DEM
{

class InsertSphereCSV : public InsertDeleteBase
{
    /*

    Inserts spheres once at a specified time.

    Variables
    =========
    spheregroup_in : SphereGroup
        Sphere group to be inserted to.
    ts_insert_in : int
        Timestep at which insertion is performed.
    file_in_str_in : string
        Path to STL file with mesh to insert.
    scale_factor_in : double
        Scale factor multiplied to the coordinates.
        Default value is 1.
    distance_verlet_rel_in : double
        Verlet distance relative to the particle radius.
        Default value is 0.50.

    Functions
    =========
    get_group_ptr_vec : vector<BaseGroup*>
        Returns pointers to group objects affected by this object.
    update : void
        Updates this object.

    Notes
    =====
    The CSV file with point data must have the following columns:
        sphere ID
        material ID
        x-coordinate of position
        y-coordinate of position
        z-coordinate of position
        x-coordinate of velocity
        y-coordinate of velocity
        z-coordinate of velocity
        x-coordinate of angular position
        y-coordinate of angular position
        z-coordinate of angular position
        x-coordinate of angular velocity
        y-coordinate of angular velocity
        z-coordinate of angular velocity
        radius
    Scale factor is multiplied to the linear positions, velocities, and radii.

    */

    public:

    // sphere group
    SphereGroup* spheregroup_ptr;

    // insertion parameters
    int ts_insert = 0;
    std::string file_in_str;
    double scale_factor = 0.;
    double distance_verlet_rel = 0.;

    // functions
    std::vector<BaseGroup*> get_group_ptr_vec() {return {spheregroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    InsertSphereCSV() {}

    // constructor
    InsertSphereCSV(SphereGroup &spheregroup_in, int ts_insert_in, std::string file_in_str_in, double scale_factor_in = 1., double distance_verlet_rel_in = 0.5)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        ts_insert = ts_insert_in;
        file_in_str = file_in_str_in;
        scale_factor = scale_factor_in;
        distance_verlet_rel = distance_verlet_rel_in;

    }

    private:

};

void InsertSphereCSV::update(int ts, double dt)
{
    /*

    Updates this object.

    Arguments
    =========
    ts : int
        Timestep number.
    dt : double
        Duration of timestep.

    Returns
    =======
    (none)

    */

    // skip if not insertion timestep
    if (ts != ts_insert)
    {
        return;
    }

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

    // initialize for iteration
    bool is_header = true;  // true while reading header
    std::string line_str;  // stores lines in files

    // iterate for each line in the file
    while (std::getline(file_in_stream, line_str))
    {

        // skip header
        if (is_header)
        {
            is_header = false; // not reading header
            continue;
        }

        // convert line string into stringstream
        std::stringstream line_stream(line_str);
    
        // initialize for iteration
        int value_num = 0;  // counts position of value
        std::string value_str;  // stores values in lines
        Sphere sphere_sub;  // shpere object

        // iterate through each value
        while (std::getline(line_stream, value_str, ','))
        {

            // store values for sphere
            switch (value_num)
            {
                case 0: sphere_sub.gid = spheregroup_ptr->num_sphere_max; break;
                case 1: sphere_sub.mid = std::stoi(value_str); break;
                case 2: sphere_sub.position.coeffRef(0) = scale_factor * std::stod(value_str); break;
                case 3: sphere_sub.position.coeffRef(1) = scale_factor * std::stod(value_str); break;
                case 4: sphere_sub.position.coeffRef(2) = scale_factor * std::stod(value_str); break;
                case 5: sphere_sub.velocity.coeffRef(0) = scale_factor * std::stod(value_str); break;
                case 6: sphere_sub.velocity.coeffRef(1) = scale_factor * std::stod(value_str); break;
                case 7: sphere_sub.velocity.coeffRef(2) = scale_factor * std::stod(value_str); break;
                case 8: sphere_sub.angularposition.coeffRef(0) = std::stod(value_str); break;
                case 9: sphere_sub.angularposition.coeffRef(1) = std::stod(value_str); break;
                case 10: sphere_sub.angularposition.coeffRef(2) = std::stod(value_str); break;
                case 11: sphere_sub.angularvelocity.coeffRef(0) = std::stod(value_str); break;
                case 12: sphere_sub.angularvelocity.coeffRef(1) = std::stod(value_str); break;
                case 13: sphere_sub.angularvelocity.coeffRef(2) = std::stod(value_str); break;
                case 14: sphere_sub.radius = scale_factor * std::stod(value_str); break;
            }

            // compute hypothetical previous position
            // assume zero acceleration
            sphere_sub.previous_position = sphere_sub.position - sphere_sub.velocity*dt;
            sphere_sub.previous_angularposition = sphere_sub.angularposition - sphere_sub.angularvelocity*dt;

            // initialize collision check data
            sphere_sub.distance_traveled = 0.;
            sphere_sub.distance_verlet = distance_verlet_rel * sphere_sub.radius;

            // increment value count
            value_num++;

        }
    
        // update mapping
        spheregroup_ptr->tid_to_gid_vec.push_back(spheregroup_ptr->num_sphere_max);
        spheregroup_ptr->gid_to_tid_map[spheregroup_ptr->num_sphere_max] = spheregroup_ptr->num_sphere;

        // insert spheres
        spheregroup_ptr->sphere_vec.push_back(sphere_sub);
        spheregroup_ptr->num_sphere += 1;
        spheregroup_ptr->num_sphere_max += 1;
        
    }

    // close file
    file_in_stream.close();

}

}

#endif
