#ifndef INSERT_SPHERE_CSV
#define INSERT_SPHERE_CSV
#include <fstream>
#include <sstream>
#include <vector>
#include "container_sphere.hpp"

class InsertSphereCSV
{

    public:

    // variables
    std::string file_in_str;
    double scale_length;

    // functions
    void insert_sphere(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    InsertSphereCSV()
    {

    }

    // constructor
    InsertSphereCSV(std::string file_in_str_in, double scale_length_in = 1.)
    {
        file_in_str = file_in_str_in;
        scale_length = scale_length_in;
    }

};

void InsertSphereCSV::insert_sphere(SpherePositionVelocityStruct &sphere_pvs)
{

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

    // initialize for iteration
    int num_particle_new = 0;
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

        // count number of particles
        sphere_pvs.num_particle++;
        num_particle_new++;

        // convert line string into stringstream
        std::stringstream line_stream(line_str);
    
        // initialize for iteration
        int value_num = 0;  // counts position of value
        std::string value_str;  // stores values in lines

        // iterate through each value
        while (std::getline(line_stream, value_str, ','))
        {

            // store values in appropriate vector
            switch (value_num)
            {
                case 0: sphere_pvs.id_vec.push_back(sphere_pvs.num_particle_historical_max + std::stoi(value_str)); break;
                case 1: sphere_pvs.type_vec.push_back(std::stoi(value_str)); break;
                case 2: sphere_pvs.position_x_vec.push_back(scale_length * std::stod(value_str)); break;
                case 3: sphere_pvs.position_y_vec.push_back(scale_length * std::stod(value_str)); break;
                case 4: sphere_pvs.position_z_vec.push_back(scale_length * std::stod(value_str)); break;
                case 5: sphere_pvs.velocity_x_vec.push_back(scale_length * std::stod(value_str)); break;
                case 6: sphere_pvs.velocity_y_vec.push_back(scale_length * std::stod(value_str)); break;
                case 7: sphere_pvs.velocity_z_vec.push_back(scale_length * std::stod(value_str)); break;
                case 8: sphere_pvs.angularposition_x_vec.push_back(std::stod(value_str)); break;
                case 9: sphere_pvs.angularposition_y_vec.push_back(std::stod(value_str)); break;
                case 10: sphere_pvs.angularposition_z_vec.push_back(std::stod(value_str)); break;
                case 11: sphere_pvs.angularvelocity_x_vec.push_back(std::stod(value_str)); break;
                case 12: sphere_pvs.angularvelocity_y_vec.push_back(std::stod(value_str)); break;
                case 13: sphere_pvs.angularvelocity_z_vec.push_back(std::stod(value_str)); break;
            }

            // increment value count
            value_num++;

        }
    
    }

    // update number of particles present in simulation
    sphere_pvs.num_particle_historical_max += num_particle_new;

    // close file
    file_in_stream.close();

}

#endif
