#ifndef INSERT_WALLMESH_STL
#define INSERT_WALLMESH_STL
#include <fstream>
#include <sstream>
#include <vector>
#include "container_wallmesh.hpp"

class InsertWallMeshSTL
{
    /*

    Inserts a wall following positions indicated by an STL file.

    Variables
    =========
    file_in_str_in : string
        File name of the CSV file.
    type_int_in : int
        Type of wall.
    scale_length_in : double
        Factor by which the length dimension is scaled.

    Functions
    =========
    insert_wallmesh : void
        Inserts a wall into the simulation.

    */

    public:

    // variables
    std::string file_in_str;
    int type_int;
    double scale_length;

    // functions
    void insert_wallmesh(WallMeshPositionVelocityStruct &wallmesh_pvs);

    // default constructor
    InsertWallMeshSTL()
    {

    }

    // constructor
    InsertWallMeshSTL(std::string file_in_str_in, int type_int_in, double scale_length_in = 1.)
    {
        file_in_str = file_in_str_in;
        type_int = type_int_in;
        scale_length = scale_length_in;
    }

};

void InsertWallMeshSTL::insert_wallmesh(WallMeshPositionVelocityStruct &wallmesh_pvs)
{
    /*

    Inserts a wall into the simulation.

    Arguments
    =========
    wallmesh_pvs : WallMeshPositionVelocityStruct
        struct with position and velocity of mesh triangles in a wall.

    Returns
    =======
    (none)

    */

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

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
            wallmesh_pvs.num_mesh++;  // increment number of triangles
            wallmesh_pvs.num_mesh_historical_max++;  // increment historical max of number of triangles
            wallmesh_pvs.id_vec.push_back(line_num/7);  // store ID of triangle
            wallmesh_pvs.type_vec.push_back(type_int);  // store type of material
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
                    case 1: wallmesh_pvs.position_p1_x_vec.push_back(scale_length * std::stod(value_str)); break; // x
                    case 2: wallmesh_pvs.position_p1_y_vec.push_back(scale_length * std::stod(value_str)); break; // y
                    case 3: wallmesh_pvs.position_p1_z_vec.push_back(scale_length * std::stod(value_str)); break; // z
                }
                break;

                case 4:  // point 2
                switch(value_num)  // each value contains data for each x, y, or z coordinate
                {
                    case 1: wallmesh_pvs.position_p2_x_vec.push_back(scale_length * std::stod(value_str)); break; // x
                    case 2: wallmesh_pvs.position_p2_y_vec.push_back(scale_length * std::stod(value_str)); break; // y
                    case 3: wallmesh_pvs.position_p2_z_vec.push_back(scale_length * std::stod(value_str)); break; // z
                }
                break;

                case 5:  // point 3
                switch(value_num)  // each value contains data for each x, y, or z coordinate
                {
                    case 1: wallmesh_pvs.position_p3_x_vec.push_back(scale_length * std::stod(value_str)); break; // x
                    case 2: wallmesh_pvs.position_p3_y_vec.push_back(scale_length * std::stod(value_str)); break; // y
                    case 3: wallmesh_pvs.position_p3_z_vec.push_back(scale_length * std::stod(value_str)); break; // z
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

}

#endif
