#ifndef INITIALIZE_SPHERE_POSITIONVELOCITY_CSV
#define INITIALIZE_SPHERE_POSITIONVELOCITY_CSV
#include <fstream>
#include <sstream>
#include <vector>
#include "container_sphere.hpp"

SpherePositionVelocityStruct initialize_sphere_positionvelocity_csv(std::string file_in_str, double scale_length = 1.)
{

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

    // initialize struct with position and velocity data
    SpherePositionVelocityStruct sphere_pvs;

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

        // count number of particles
        sphere_pvs.num_particle++;
        sphere_pvs.num_particle_historical_max++;

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
                case 0: sphere_pvs.id_vec.push_back(std::stoi(value_str)); break;
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

    // close file
    file_in_stream.close();

    return sphere_pvs;

}

#endif
