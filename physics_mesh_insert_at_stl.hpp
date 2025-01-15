#ifndef PHYSICS_MESH_INSERT_AT_STL
#define PHYSICS_MESH_INSERT_AT_STL
#include <fstream>
#include <set>
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

    // data from stl file
    int num_face = 0;
    std::vector<EigenVector3D> position_p1_vec;
    std::vector<EigenVector3D> position_p2_vec;
    std::vector<EigenVector3D> position_p3_vec;

    // data to insert
    int material = 0;
    int num_edge = 0;
    int num_vertex = 0;
    VectorInt2D fid_lid_to_pid_vec;
    VectorInt2D eid_lid_to_pid_vec;
    std::vector<EigenVector3D> position_vec;

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

        // read stl file then generate data to insert
        read_stl();
        generate_face_edge_vertex();

    }

    private:
    
    // functions
    void read_stl();
    void generate_face_edge_vertex();

};

void PhysicsMeshInsertAtSTL::compute_insert_delete(int ts)
{

    // skip if no insertion
    if (ts != ts_insert)
    {
        return;
    }

    // set wall data
    mesh_ptr->num_face = num_face;
    mesh_ptr->num_edge = num_edge;
    mesh_ptr->num_vertex = num_vertex;
    mesh_ptr->fid_lid_to_pid_vec = fid_lid_to_pid_vec;
    mesh_ptr->eid_lid_to_pid_vec = eid_lid_to_pid_vec;
    mesh_ptr->material = material;
    mesh_ptr->position_vec = position_vec;

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
            num_face++;  // increment number of triangles
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

void PhysicsMeshInsertAtSTL::generate_face_edge_vertex()
{

    // initialize
    std::set<VectorInt> eid_lid_to_pid_set;

    // iterate through each face
    for (int indx_f = 0; indx_f < num_face; indx_f++)
    {

        // get points in face
        EigenVector3D position_p1 = position_p1_vec[indx_f];
        EigenVector3D position_p2 = position_p2_vec[indx_f];
        EigenVector3D position_p3 = position_p3_vec[indx_f];
        
        // find point in vector if it exists
        auto iter_p1 = std::find(position_vec.begin(), position_vec.end(), position_p1);
        auto iter_p2 = std::find(position_vec.begin(), position_vec.end(), position_p2);
        auto iter_p3 = std::find(position_vec.begin(), position_vec.end(), position_p3);
        bool is_p1_not_found = (iter_p1 == position_vec.end());
        bool is_p2_not_found = (iter_p2 == position_vec.end());
        bool is_p3_not_found = (iter_p3 == position_vec.end());

        // get index of points in vector
        int pid_end = position_vec.size();
        int pid_p1 = iter_p1 - position_vec.begin();
        int pid_p2 = iter_p2 - position_vec.begin();
        int pid_p3 = iter_p3 - position_vec.begin();
        if (is_p1_not_found) {pid_p1 = pid_end; position_vec.push_back(position_p1); pid_end++;}
        if (is_p2_not_found) {pid_p2 = pid_end; position_vec.push_back(position_p2); pid_end++;}
        if (is_p3_not_found) {pid_p3 = pid_end; position_vec.push_back(position_p3); pid_end++;}

        // store indices of points on face
        fid_lid_to_pid_vec.push_back({pid_p1, pid_p2, pid_p3});

        // create pairs of edges
        VectorInt pid_p1p2 = {pid_p1, pid_p2};
        VectorInt pid_p2p3 = {pid_p2, pid_p3};
        VectorInt pid_p3p1 = {pid_p3, pid_p1};

        // ensure that first ID is smaller for consistency
        // needed for duplicate checking
        if (pid_p1 > pid_p2) {pid_p1p2 = {pid_p2, pid_p1};}
        if (pid_p2 > pid_p3) {pid_p2p3 = {pid_p3, pid_p2};}
        if (pid_p3 > pid_p1) {pid_p3p1 = {pid_p1, pid_p3};}

        // insert in set of edges
        eid_lid_to_pid_set.insert(pid_p1p2);
        eid_lid_to_pid_set.insert(pid_p2p3);
        eid_lid_to_pid_set.insert(pid_p3p1);

    }

    // convert to set
    eid_lid_to_pid_vec = VectorInt2D(eid_lid_to_pid_set.begin(), eid_lid_to_pid_set.end());

    // count edges and vertices
    num_edge = eid_lid_to_pid_vec.size();
    num_vertex = position_vec.size();

}

}

#endif
