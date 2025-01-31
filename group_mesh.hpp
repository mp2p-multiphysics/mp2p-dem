#ifndef GROUP_MESH
#define GROUP_MESH
#include <fstream>
#include <sstream>
#include <vector>
#include "container_typedef.hpp"
#include "group_base.hpp"

namespace DEM
{

struct Mesh
{

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // IDs
    int gid; // group ID

    // point IDs
    int pid_p0;
    int pid_p1;
    int pid_p2;

    // positions (current step)
    EigenVector3D position_p0 = EigenVector3D::Zero();
    EigenVector3D position_p1 = EigenVector3D::Zero();
    EigenVector3D position_p2 = EigenVector3D::Zero();

};

class MeshGroup : public BaseGroup
{
    /*

    Walls represented as triangular elements.

    Functions
    =========
    set_output_position : void
        Set the output STL file name with position values.
    set_output_forcemoment : void
        Set the output CSV file name with force and moment values.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // number of mesh triangles
    int num_mesh = 0;

    // vector of meshes
    std::vector<Mesh, Eigen::aligned_allocator<Mesh>> mesh_vec;
    std::vector<EigenVector3D> point_vec;

    // output file
    std::string file_out_position_str;
    std::string file_out_forcemoment_str;
    std::ofstream log_forcemoment_stream;

    // material ID
    int mid = 0;

    // velocity
    EigenVector3D velocity_translate = EigenVector3D::Zero();
    double angularvelocity_rotate = 0;
    EigenVector3D position_rotateaxis_begin = EigenVector3D::Zero();
    EigenVector3D position_rotateaxis_end = EigenVector3D::Zero();

    // force and moment
    EigenVector3D force = EigenVector3D::Zero();
    EigenVector3D moment = EigenVector3D::Zero();
    EigenVector3D position_ref = EigenVector3D::Zero();  // reference point for moment calculation

    // functions
    void output_file(int ts);
    void set_output_position(std::string file_out_str);
    void set_output_forcemoment(std::string file_out_str, EigenVector3D position_ref);

    // default constructor
    MeshGroup() {}

    private:

    // functions
    void output_position_stl(int ts);
    void output_forcemoment_csv(int ts);

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
    output_forcemoment_csv(ts);

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

void MeshGroup::set_output_forcemoment(std::string file_out_str, EigenVector3D position_ref_in)
{
    /*

    Set the output CSV file name with force and moment values.

    Arguments
    =========
    file_out_str : string
        Path to STL file.
    position_ref : EigenVector3D
        Reference point for moment calculation.

    Returns
    =======
    (none)

    */
    
    // store inputs
    file_out_forcemoment_str = file_out_str;
    position_ref = position_ref_in;

    // initialize log file with force and moment
    log_forcemoment_stream.open(file_out_forcemoment_str);
    log_forcemoment_stream << "ts,force_x,force_y,force_z,moment_x,moment_y,moment_z\n";
    log_forcemoment_stream.close();
    log_forcemoment_stream.open(file_out_forcemoment_str, std::ios_base::app | std::ios_base::out);

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

void MeshGroup::output_forcemoment_csv(int ts)
{

    // do not make file if filename not set
    if (file_out_position_str.empty())
    {
        return;
    }

    // write to file
    log_forcemoment_stream << ts << ",";
    log_forcemoment_stream << force(0) << ",";
    log_forcemoment_stream << force(1) << ",";
    log_forcemoment_stream << force(2) << ",";
    log_forcemoment_stream << moment(0) << ",";
    log_forcemoment_stream << moment(1) << ",";
    log_forcemoment_stream << moment(2) << "\n";

}

}

#endif
