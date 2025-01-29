#ifndef GROUP_MESH
#define GROUP_MESH
#include <fstream>
#include <sstream>
#include "container_typedef.hpp"
#include "group_base.hpp"

namespace DEM
{

struct Mesh
{

    // IDs
    int gid; // group ID

    // point IDs
    int pid_p0;
    int pid_p1;
    int pid_p2;

    // positions (current step)
    EigenVector3D position_p0;
    EigenVector3D position_p1;
    EigenVector3D position_p2;

};

class MeshGroup : public BaseGroup
{
    /*

    Walls represented as triangular elements.

    Functions
    =========
    set_output_position : void
        Set the output STL file name with position values.

    */

    public:

    // number of mesh triangles
    int num_mesh = 0;

    // vector of meshes
    std::vector<Mesh> mesh_vec;
    std::vector<EigenVector3D> point_vec;

    // output file
    std::string file_out_position_str;

    // material ID
    int mid = 0;

    // velocity
    EigenVector3D velocity_translate;
    double angularvelocity_rotate = 0;
    EigenVector3D position_rotateaxis_begin;
    EigenVector3D position_rotateaxis_end;

    // functions
    void output_file(int ts);
    void set_output_position(std::string file_out_str);

    // default constructor
    MeshGroup() {}

    private:

    // functions
    void output_position_stl(int ts);

};

void MeshGroup::output_file(int ts)
{
    /*

    Writes output files.

    Arguments
    =========
    ts : int
        Timestep number.

    Returns
    =======
    (none)

    */

    // generate output files
    output_position_stl(ts);

}

void MeshGroup::set_output_position(std::string file_out_str)
{
    /*

    Set the output STL file name with position values.

    Arguments
    =========
    file_out_str : string
        Path to STL file.

    Returns
    =======
    (none)

    Notes
    =====
    file_out_str must have an asterisk '*'.
    This will be replaced with the timestep number.

    */
    
    file_out_position_str = file_out_str;

}

void MeshGroup::output_position_stl(int ts)
{

    // do not make file if filename not set
    if (file_out_position_str.empty())
    {
        return;
    }

    // split filename at '*'
    // will be replaced with timestep later
    std::vector<std::string> file_out_base_vec;
    std::stringstream file_out_base_stream(file_out_position_str);
    std::string string_sub;
    while(std::getline(file_out_base_stream, string_sub, '*'))
    {
        file_out_base_vec.push_back(string_sub);
    }

    // create output filename
    // replace '*' with timestep
    std::string file_out_str = file_out_base_vec[0];
    for (int i = 1; i < file_out_base_vec.size(); i++)
    {
        file_out_str += std::to_string(ts) + file_out_base_vec[i];
    }

    // initialize file stream
    std::ofstream file_out_stream(file_out_str);

    // write output data
    file_out_stream << "solid mp2p\n";
    for (auto &mesh : mesh_vec)
    {

        // get wall points
        EigenVector3D pos_p0 = mesh.position_p0;
        EigenVector3D pos_p1 = mesh.position_p1;
        EigenVector3D pos_p2 = mesh.position_p2;

        // calculate front facing normal vectors
        EigenVector3D front = (pos_p1 - pos_p0).cross(pos_p2 - pos_p0);
        EigenVector3D norm = front/front.norm();

        // write lines in file
        file_out_stream << "facet normal " << norm(0) << " " << norm(1)  << " " << norm(2) << "\n";
        file_out_stream << "  outer loop\n";
        file_out_stream << "    vertex " << pos_p0(0) << " " << pos_p0(1) << " " << pos_p0(2) << "\n";
        file_out_stream << "    vertex " << pos_p1(0) << " " << pos_p1(1) << " " << pos_p1(2) << "\n";
        file_out_stream << "    vertex " << pos_p2(0) << " " << pos_p2(1) << " " << pos_p2(2) << "\n";
        file_out_stream << "  endloop\n";
        file_out_stream << "endfacet\n";

    }
    file_out_stream << "endsolid mp2p\n";

    // close
    file_out_stream.close();

}

}

#endif
