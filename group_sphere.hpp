#ifndef GROUP_SPHERE
#define GROUP_SPHERE
#include <fstream>
#include <sstream>
#include "container_typedef.hpp"
#include "group_base.hpp"

namespace DEM
{

struct Sphere
{

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // IDs
    int gid; // group ID
    int mid; // material ID

    // forces and moments
    EigenVector3D force = EigenVector3D::Zero();
    EigenVector3D moment = EigenVector3D::Zero();

    // positions and velocities (current step)
    EigenVector3D position = EigenVector3D::Zero();
    EigenVector3D velocity = EigenVector3D::Zero();
    EigenVector3D angularposition = EigenVector3D::Zero();
    EigenVector3D angularvelocity = EigenVector3D::Zero();

    // positions (previous step)
    EigenVector3D previous_position = EigenVector3D::Zero();
    EigenVector3D previous_angularposition = EigenVector3D::Zero();

    // geometry
    double radius;

};

class SphereGroup : public BaseGroup
{
    /*

    Particles represented as spheres.

    Functions
    =========
    set_output_position : void
        Set the output CSV file name with position and velocity values.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // number of spheres
    int num_sphere = 0;  // current number
    int num_sphere_max = 0;  // historical max

    // vector of spheres
    std::vector<Sphere, Eigen::aligned_allocator<Sphere>> sphere_vec;
    VectorInt tid_to_gid_vec;  // maps temporary ID (index in sphere_vec) to group ID
    MapIntInt gid_to_tid_map;  // maps group ID to temporary ID (index in sphere_vec)

    // output file
    std::string file_out_positionvelocity_str;

    // functions
    void output_file(int ts);
    void set_output_positionvelocity(std::string file_out_str);
    
    // default constructor
    SphereGroup() {}

    private:

    // functions
    void output_positionvelocity_csv(int ts);

};

void SphereGroup::output_file(int ts)
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
    output_positionvelocity_csv(ts);

}

void SphereGroup::set_output_positionvelocity(std::string file_out_str)
{
    /*

    Set the output CSV file name with position and velocity values.

    Arguments
    =========
    file_out_str : string
        Path to CSV file.

    Returns
    =======
    (none)

    Notes
    =====
    file_out_str must have an asterisk '*'.
    This will be replaced with the timestep number.

    */

    file_out_positionvelocity_str = file_out_str;

}

void SphereGroup::output_positionvelocity_csv(int ts)
{

    // do not make file if filename not set
    if (file_out_positionvelocity_str.empty())
    {
        return;
    }

    // split filename at '*'
    // will be replaced with timestep later
    std::vector<std::string> file_out_base_vec;
    std::stringstream file_out_base_stream(file_out_positionvelocity_str);
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

    // write to file
    file_out_stream << "id,material,position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,angularposition_x,angularposition_y,angularposition_z,angularvelocity_x,angularvelocity_y,angularvelocity_z,radius\n";
    for (auto &sphere : sphere_vec)
    {
        file_out_stream << sphere.gid << ",";
        file_out_stream << sphere.mid << ",";
        file_out_stream << sphere.position.coeffRef(0) << ",";
        file_out_stream << sphere.position.coeffRef(1) << ",";
        file_out_stream << sphere.position.coeffRef(2) << ",";
        file_out_stream << sphere.velocity.coeffRef(0) << ",";
        file_out_stream << sphere.velocity.coeffRef(1) << ",";
        file_out_stream << sphere.velocity.coeffRef(2) << ",";
        file_out_stream << sphere.angularposition.coeffRef(0) << ",";
        file_out_stream << sphere.angularposition.coeffRef(1) << ",";
        file_out_stream << sphere.angularposition.coeffRef(2) << ",";
        file_out_stream << sphere.angularvelocity.coeffRef(0) << ",";
        file_out_stream << sphere.angularvelocity.coeffRef(1) << ",";
        file_out_stream << sphere.angularvelocity.coeffRef(2) << ",";
        file_out_stream << sphere.radius << "\n";
    }

}

}

#endif
