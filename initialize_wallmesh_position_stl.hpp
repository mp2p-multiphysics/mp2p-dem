#ifndef INITIALIZE_WALLMESH_POSITION_STL
#define INITIALIZE_WALLMESH_POSITION_STL
#include <fstream>
#include <sstream>
#include <vector>
#include "container_wallmesh.hpp"

WallMeshPositionVelocityStruct initialize_wallmesh_position_stl(std::string file_in_str, int type_int, double scale_length = 1.)
{

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

    // initialize struct with coordinates of mesh triangles
    WallMeshPositionVelocityStruct wpvs;

    // iterate for each line
    int line_num = 0;  // counts line numbers
    std::string line_str;  // stores lines in files

    // iterate for each line in the file
    while (std::getline(file_in_stream, line_str))
    {

        // skip lines without coordinate data
        // every 0, 1, 2, and 6th line has no data
        if (line_num % 7 == 0 || line_num % 7 == 1 || line_num % 7 == 2 || line_num % 7 == 6)
        {
            line_num++;  // increment line number
            continue;
        }

        // update data for each mesh triangle
        // every 3rd line starts the triangle data
        if (line_num % 7 == 3)
        {
            wpvs.num_mesh++;  // increment number of triangles
            wpvs.id_vec.push_back(line_num/7);  // store ID of triangle
            wpvs.type_vec.push_back(type_int);  // store type of material
        }

        // convert line string into stringstream
        std::stringstream line_stream(line_str);
    
        // initialize for iteration
        int value_num = 0;  // counts position of value
        std::string value_str;  // stores values in lines

        // iterate through each value
        while (std::getline(line_stream, value_str, ' '))
        {
            
            // skip empty lines
            if (value_str == "")
            {
                continue;
            }

            // store values in appropriate vector
            switch(line_num % 7)  // each line contains data for each point
            {
                
                case 3:  // point 1
                switch(value_num)  // each value contains data for each x, y, or z coordinate
                {
                    case 1: wpvs.position_p1_x_vec.push_back(scale_length * std::stod(value_str)); break; // x
                    case 2: wpvs.position_p1_y_vec.push_back(scale_length * std::stod(value_str)); break; // y
                    case 3: wpvs.position_p1_z_vec.push_back(scale_length * std::stod(value_str)); break; // z
                }
                break;

                case 4:  // point 2
                switch(value_num)  // each value contains data for each x, y, or z coordinate
                {
                    case 1: wpvs.position_p2_x_vec.push_back(scale_length * std::stod(value_str)); break; // x
                    case 2: wpvs.position_p2_y_vec.push_back(scale_length * std::stod(value_str)); break; // y
                    case 3: wpvs.position_p2_z_vec.push_back(scale_length * std::stod(value_str)); break; // z
                }
                break;

                case 5:  // point 3
                switch(value_num)  // each value contains data for each x, y, or z coordinate
                {
                    case 1: wpvs.position_p3_x_vec.push_back(scale_length * std::stod(value_str)); break; // x
                    case 2: wpvs.position_p3_y_vec.push_back(scale_length * std::stod(value_str)); break; // y
                    case 3: wpvs.position_p3_z_vec.push_back(scale_length * std::stod(value_str)); break; // z
                }
                break;

            }
            
            // increment value count
            value_num++;

        }

        // increment line number
        line_num++;
    
    }

    // close file
    file_in_stream.close();

    return wpvs;

}

#endif
