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

    // group ID
    int gid;

    // material ID
    int mid;

    // positions and velocities
    EigenVector3D position;
    EigenVector3D velocity;
    EigenVector3D angularposition;
    EigenVector3D angularvelocity;

    // forces and moments
    EigenVector3D force;
    EigenVector3D moment;

    // geometry
    double radius;

    // enlarged geometry for collision detection
    double radius_enlarged;

};

class GroupSphere : public GroupBase
{

    public:

    // largest inserted ID so far
    int gid_max = 0;

    // vector of spheres
    std::vector<Sphere> sphere_vec;

    // output file
    std::string file_out_positionvelocity_str;

    // functions
    void write_output(int ts);
    void clear_forcemoment();
    void set_output_positionvelocity(std::string file_out_str);

    // default constructor
    GroupSphere() {}

    private:

    // functions
    void output_positionvelocity_csv(int ts);

};

void GroupSphere::clear_forcemoment()
{

    // iterate through each sphere
    for (auto &sphere : sphere_vec)
    {
        sphere.force = {0., 0., 0.,};
        sphere.moment = {0., 0., 0.,};
    }

}

void GroupSphere::write_output(int ts)
{

    // generate output files
    output_positionvelocity_csv(ts);

}

void GroupSphere::set_output_positionvelocity(std::string file_out_str)
{
    file_out_positionvelocity_str = file_out_str;
}

void GroupSphere::output_positionvelocity_csv(int ts)
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
    file_out_stream << "sphere_id,position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,position_x,angularposition_y,angularposition_z,angularvelocity_x,angularvelocity_y,angularvelocity_z\n";
    for (auto &sphere : sphere_vec)
    {
        file_out_stream << sphere.gid << ",";
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
        file_out_stream << sphere.angularvelocity.coeffRef(2) << "\n";
    }

}

}

#endif
