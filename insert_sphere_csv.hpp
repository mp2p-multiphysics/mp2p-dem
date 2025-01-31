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

    Inserts spheres from a CSV file at specified times.

    Variables
    =========
    spheregroup_in : SphereGroup
        Sphere group to be inserted to.
    file_in_str_in : string
        Path to STL file with mesh to insert.
    ts_start_in : int
        Timestep at which particle insertion starts.
    ts_end_in : int
        Timestep at which particle insertion ends.
        No insertion is made at and after this timestep.
        Set to -1 for no end.
        Default value is -1.
    ts_step_in : int
        Time interval between insertions.
        Set to -1 to insert particles only once.
        Default value is -1.
    scale_factor_in : double
        Scale factor multiplied to the coordinates.
        Default value is 1.

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

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // sphere group
    double dt = 0.;
    SphereGroup* spheregroup_ptr;

    // insertion parameters
    std::string file_in_str;
    double scale_factor = 0.;

    // timesteps
    int ts_start = 0;
    int ts_end = 0;
    int ts_step = 0;

    // spheres to insert
    std::vector<Sphere> sphere_vec;

    // functions
    void initialize(double dt_in);
    void update(int ts);

    // default constructor
    InsertSphereCSV() {}

    // constructor
    InsertSphereCSV(SphereGroup &spheregroup_in, std::string file_in_str_in, int ts_start_in, int ts_end_in = -1, int ts_step_in = -1, double scale_factor_in = 1.)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        file_in_str = file_in_str_in;

        // store timesteps
        ts_start = ts_start_in;
        ts_end = ts_end_in;
        ts_step = ts_step_in;

        // store other optional inputs
        scale_factor = scale_factor_in;

    }

    private:

};

void InsertSphereCSV::update(int ts)
{
    /*

    Update this object.

    Arguments
    =========
    ts : int
        Timestep number.

    Returns
    =======
    (none)

    */

    // check if insertion is needed
    if (ts < ts_start)  // insertion has not started yet
    {
        return;  
    }
    if (ts_end != -1 && ts >= ts_end)  // insertion has ended
    {
        return;  
    }
    if (ts_step == -1)
    {
        if (ts != ts_start)  // only insert once at ts_start_in
        {
            return;  
        }
    }
    else if ((ts - ts_start) % ts_step != 0)  // insert only at defined intervals
    {
        return;
    }

    // insert spheres
    for (auto sphere : sphere_vec)
    {
        sphere.gid = spheregroup_ptr->num_sphere_max;
        spheregroup_ptr->tid_to_gid_vec.push_back(spheregroup_ptr->num_sphere_max);
        spheregroup_ptr->gid_to_tid_map[spheregroup_ptr->num_sphere_max] = spheregroup_ptr->num_sphere;
        spheregroup_ptr->sphere_vec.push_back(sphere);
        spheregroup_ptr->num_sphere++;
        spheregroup_ptr->num_sphere_max++;
    }

}

void InsertSphereCSV::initialize(double dt_in)
{
    /*

    Initialize this object.

    Arguments
    =========
    dt_in : double
        Duration of timestep.

    Returns
    =======
    (none)

    */

    // store timestep
    dt = dt_in;

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
                case 0: sphere_sub.gid = -1; break;
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

            // increment value count
            value_num++;

        }
    
        // insert spheres
        sphere_vec.push_back(sphere_sub);
        
    }

    // close file
    file_in_stream.close();

}

}

#endif
