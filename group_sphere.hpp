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

class SphereGroup : public BaseGroup
{

    public:

    // number of spheres
    int num_sphere = 0;  // current number
    int num_sphere_max = 0;  // historical max

    // vector of spheres
    // note: order of spheres in sphere_vec must match sphere_previous_ts_vec
    std::vector<Sphere> sphere_vec;
    std::vector<Sphere> sphere_previous_ts_vec;

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
    output_positionvelocity_csv(ts);
}

void SphereGroup::set_output_positionvelocity(std::string file_out_str)
{
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
