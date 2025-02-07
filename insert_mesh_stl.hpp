#ifndef INSERT_MESH_STL
#define INSERT_MESH_STL
#include <algorithm>
#include <fstream>
#include <sstream>
#include "insertdelete_base.hpp"
#include "group_mesh.hpp"

namespace DEM
{

class InsertMeshSTL : public InsertDeleteBase
{
    /*

    Inserts a mesh from an STL file at a specified time.

    Variables
    =========
    meshgroup_in : MeshGroup
        Mesh group to be inserted to.
    mid_in : int
        Material ID.
    file_in_str_in : string
        Path to STL file with mesh to insert.
    ts_insert_in : int
        Timestep at which the mesh is inserted.
    velocity_translate_in : Eigen::Vector3d
        Translational velocity of the mesh.
        Default value is {0, 0, 0}.
    angularvelocity_rotate_in
        Rotational velocity of the mesh around an axis.
        Default value is 0.
    position_rotateaxis_begin_in : Eigen::Vector3d
        Start point of the axis of rotation.
        Default value is {0, 0, 0}.
    position_rotateaxis_end_in : Eigen::Vector3d
        End point of the axis of rotation.
        Default value is {0, 0, 1}.
    scale_factor_in : double
        Scale factor multiplied to the coordinates.
        Default value is 1.

    Notes
    =====
    Direction of rotation follows the right-hand rule.
    Scale factor is multiplied to the positions of each point in the mesh.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // mesh group
    double dt = 0.;
    MeshGroup* meshgroup_ptr;

    // insertion parameters
    std::string file_in_str;
    double scale_factor = 0.;

    // material ID
    int mid = 0;

    // timesteps
    int ts_insert = 0;

    // velocity
    EigenVector3D velocity_translate = EigenVector3D::Zero();
    double angularvelocity_rotate = 0;
    EigenVector3D position_rotateaxis_begin = EigenVector3D::Zero();
    EigenVector3D position_rotateaxis_end = EigenVector3D::Zero();

    // meshes to insert
    int num_mesh = 0;
    std::vector<Mesh, Eigen::aligned_allocator<Mesh>> mesh_vec;

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    InsertMeshSTL() {}

    // constructor
    InsertMeshSTL
    (
        MeshGroup &meshgroup_in, int mid_in, std::string file_in_str_in, int ts_insert_in,
        EigenVector3D velocity_translate_in = {0., 0., 0.}, double angularvelocity_rotate_in = 0.,
        EigenVector3D position_rotateaxis_begin_in = {0., 0., 0.}, EigenVector3D position_rotateaxis_end_in = {0., 0., 1.},
        double scale_factor_in = 1.
    )
    {

        // store inputs
        meshgroup_ptr = &meshgroup_in;
        mid = mid_in;
        file_in_str = file_in_str_in;

        // store timesteps
        ts_insert = ts_insert_in;

        // store velocity
        velocity_translate = velocity_translate_in;
        angularvelocity_rotate = angularvelocity_rotate_in;
        position_rotateaxis_begin = position_rotateaxis_begin_in;
        position_rotateaxis_end = position_rotateaxis_end_in;

        // store other optional inputs
        scale_factor = scale_factor_in;

    }

    private:

};

void InsertMeshSTL::update(int ts)
{
    /*

    Updates this object.

    Arguments
    =========
    ts : int
        Timestep number.
    
    Returns
    =======
    (none)

    */

    // check if insertion is needed
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
            
            // mesh ID
            mesh_sub.gid = meshgroup_ptr->num_mesh;

            // insert mesh into group
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
