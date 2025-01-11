#ifndef PHYSICS_MESH_INSERT_AT_STL
#define PHYSICS_MESH_INSERT_AT_STL
#include <fstream>
#include <sstream>
#include <vector>
#include "physics_base.hpp"
#include "wall_mesh.hpp"

namespace DEM
{

class PhysicsMeshInsertAtSTL : public PhysicsBase
{

    public:

    // wall object to insert to
    WallMesh* mesh_ptr;
    double scale_factor = 0.;

    // insertion times
    int ts_insert = 0;

    // data aobut particles to be inserted
    int num_element = 0;
    int material = 0;
    std::vector<EigenVector3D> position_p1_vec;
    std::vector<EigenVector3D> position_p2_vec;
    std::vector<EigenVector3D> position_p3_vec;

    // file names
    std::string file_in_str;

    // functions used in timestepping
    void compute_force(int ts) {};
    void compute_position_velocity(int ts) {};
    void compute_insert_delete(int ts);

    // default constructor
    PhysicsMeshInsertAtSTL() {}

    // constructor
    PhysicsMeshInsertAtSTL(WallMesh &mesh_in, int ts_insert_in, std::string file_in_str_in, int material_in, double scale_factor_in = 1.0)
    {
        
        // store wall objects
        mesh_ptr = &mesh_in;
        material = material_in;
        scale_factor = scale_factor_in;

        // store insertion times
        ts_insert = ts_insert_in;

        // store file names
        file_in_str = file_in_str_in;

        // read stl files
        read_stl();

    }

    private:
    
    // functions
    void read_stl();

};

void PhysicsMeshInsertAtSTL::compute_insert_delete(int ts)
{

    // skip if no insertion
    if (ts != ts_insert)
    {
        return;
    }

    // set wall data
    mesh_ptr->num_element = num_element;
    mesh_ptr->material = material;
    mesh_ptr->position_p1_vec = position_p1_vec;
    mesh_ptr->position_p2_vec = position_p2_vec;
    mesh_ptr->position_p3_vec = position_p3_vec;

}

void PhysicsMeshInsertAtSTL::read_stl()
{

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
            num_element++;  // increment number of triangles
        }

        // convert line string into stringstream
        std::stringstream line_stream(line_str);
    
        // initialize for iteration
        int value_num = 0;  // counts position of value
        std::string value_str;  // stores values in lines
        EigenVector3D position_vec;  // stores point coordinates

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
                    case 1: position_vec.coeffRef(0) = scale_factor * std::stod(value_str); break; // x
                    case 2: position_vec.coeffRef(1) = scale_factor * std::stod(value_str); break; // y
                    case 3: position_vec.coeffRef(2) = scale_factor * std::stod(value_str); break; // z
                }
                position_p1_vec.push_back(position_vec);
                break;

                case 4:  // point 2
                switch(value_num)  // each value contains data for each x, y, or z coordinate
                {
                    case 1: position_vec.coeffRef(0) = scale_factor * std::stod(value_str); break; // x
                    case 2: position_vec.coeffRef(1) = scale_factor * std::stod(value_str); break; // y
                    case 3: position_vec.coeffRef(2) = scale_factor * std::stod(value_str); break; // z
                }
                position_p2_vec.push_back(position_vec);
                break;

                case 5:  // point 3
                switch(value_num)  // each value contains data for each x, y, or z coordinate
                {
                    case 1: position_vec.coeffRef(0) = scale_factor * std::stod(value_str); break; // x
                    case 2: position_vec.coeffRef(1) = scale_factor * std::stod(value_str); break; // y
                    case 3: position_vec.coeffRef(2) = scale_factor * std::stod(value_str); break; // z
                }
                position_p3_vec.push_back(position_vec);
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

}

#endif
