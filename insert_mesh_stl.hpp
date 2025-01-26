#ifndef INSERT_MESH_STL
#define INSERT_MESH_STL
#include <fstream>
#include <sstream>
#include "insertdelete_base.hpp"
#include "group_mesh.hpp"

namespace DEM
{

class InsertMeshSTL : public InsertDeleteBase
{

    public:

    // mesh group
    MeshGroup* meshgroup_ptr;

    // insertion parameters
    int ts_insert = 0;
    std::string file_in_str;
    double scale_factor = 0.;
    double enlarge_ratio = 0.;

    // material ID
    int mid = 0;

    // velocity
    EigenVector3D velocity_translate;
    double angularvelocity_rotate = 0;
    EigenVector3D position_rotateaxis_begin;
    EigenVector3D position_rotateaxis_end;

    // functions
    std::vector<BaseGroup*> get_group_ptr_vec() {return {meshgroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    InsertMeshSTL() {}

    // constructor
    InsertMeshSTL
    (
        MeshGroup &meshgroup_in, int ts_insert_in, int mid_in, std::string file_in_str_in,
        EigenVector3D velocity_translate_in = {0., 0., 0.}, double angularvelocity_rotate_in = 0.,
        EigenVector3D position_rotateaxis_begin_in = {0., 0., 0.}, EigenVector3D position_rotateaxis_end_in = {0., 0., 1.},
        double scale_factor_in = 1., double enlarge_ratio_in = 1.05
    )
    {

        // store inputs
        meshgroup_ptr = &meshgroup_in;
        ts_insert = ts_insert_in;
        mid = mid_in;
        file_in_str = file_in_str_in;

        // store velocity
        velocity_translate = velocity_translate_in;
        angularvelocity_rotate = angularvelocity_rotate_in;
        position_rotateaxis_begin = position_rotateaxis_begin_in;
        position_rotateaxis_end = position_rotateaxis_end_in;

        // store other optional inputs
        scale_factor = scale_factor_in;
        enlarge_ratio = enlarge_ratio_in;

    }

    private:

};

void InsertMeshSTL::update(int ts, double dt)
{

    // skip if not insertion timestep
    if (ts != ts_insert)
    {
        return;
    }

    // store group parameters
    meshgroup_ptr->mid = mid;
    meshgroup_ptr->velocity_translate = velocity_translate;
    meshgroup_ptr->angularvelocity_rotate = angularvelocity_rotate;
    meshgroup_ptr->position_rotateaxis_begin = position_rotateaxis_begin;
    meshgroup_ptr->position_rotateaxis_end = position_rotateaxis_end;

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

    // iterate for each line
    int line_num = 0;  // counts line numbers
    std::string line_str;  // stores lines in files

    // initialize
    Mesh mesh_sub;

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
                
                case 3:  // point 0
                {
                    // store point in mesh
                    switch(value_num)
                    {
                        case 1: mesh_sub.position_p0.coeffRef(0) = scale_factor * std::stod(value_str); break; // x
                        case 2: mesh_sub.position_p0.coeffRef(1) = scale_factor * std::stod(value_str); break; // y
                        case 3: mesh_sub.position_p0.coeffRef(2) = scale_factor * std::stod(value_str); break; // z
                    }

                    // check if point is already recorded
                    // if not, then store in point ID vector
                    auto iter = std::find(meshgroup_ptr->point_vec.begin(), meshgroup_ptr->point_vec.end(), mesh_sub.position_p0);
                    mesh_sub.pid_p0 = std::distance(meshgroup_ptr->point_vec.begin(), iter);
                    if (iter == meshgroup_ptr->point_vec.end())
                    {
                        meshgroup_ptr->point_vec.push_back(mesh_sub.position_p0);
                    }

                    break;

                }
                case 4:  // point 1
                {

                    // store point in mesh
                    switch(value_num)
                    {
                        case 1: mesh_sub.position_p1.coeffRef(0) = scale_factor * std::stod(value_str); break; // x
                        case 2: mesh_sub.position_p1.coeffRef(1) = scale_factor * std::stod(value_str); break; // y
                        case 3: mesh_sub.position_p1.coeffRef(2) = scale_factor * std::stod(value_str); break; // z
                    }

                    // check if point is already recorded
                    // if not, then store in point ID vector
                    auto iter = std::find(meshgroup_ptr->point_vec.begin(), meshgroup_ptr->point_vec.end(), mesh_sub.position_p1);
                    mesh_sub.pid_p1 = std::distance(meshgroup_ptr->point_vec.begin(), iter);
                    if (iter == meshgroup_ptr->point_vec.end())
                    {
                        meshgroup_ptr->point_vec.push_back(mesh_sub.position_p1);
                    }

                    break;

                }
                case 5:  // point 2
                {

                    // store point in mesh
                    switch(value_num)
                    {
                        case 1: mesh_sub.position_p2.coeffRef(0) = scale_factor * std::stod(value_str); break; // x
                        case 2: mesh_sub.position_p2.coeffRef(1) = scale_factor * std::stod(value_str); break; // y
                        case 3: mesh_sub.position_p2.coeffRef(2) = scale_factor * std::stod(value_str); break; // z
                    }

                    // check if point is already recorded
                    // if not, then store in point ID vector
                    auto iter = std::find(meshgroup_ptr->point_vec.begin(), meshgroup_ptr->point_vec.end(), mesh_sub.position_p2);
                    mesh_sub.pid_p2 = std::distance(meshgroup_ptr->point_vec.begin(), iter);
                    if (iter == meshgroup_ptr->point_vec.end())
                    {
                        meshgroup_ptr->point_vec.push_back(mesh_sub.position_p2);
                    }

                    break;
                
                }

            }
            
            // increment value count
            value_num++;

        }

        // update data for each mesh triangle
        // every 5th line indicates end of triangle data
        if (line_num % 7 == 5)
        {
            meshgroup_ptr->mesh_vec.push_back(mesh_sub);
            meshgroup_ptr->num_mesh++;
        }

        // increment line number
        line_num++;
    
    }

    // close file
    file_in_stream.close();

}

}

#endif
