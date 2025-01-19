#ifndef POSITIONVELOCITY_SPHERE
#define POSITIONVELOCITY_SPHERE
#include <fstream>
#include <sstream>
#include "container_typedef.hpp"
#include "sphere_group.hpp"
#include "parameter_unary.hpp"
#include "positionvelocity_base.hpp"
#include "sphere.hpp"

namespace DEM
{

class PositionVelocitySphere : public PositionVelocityBase
{

    public:

    // sphere group
    SphereGroup* spheregroup_ptr;

    // parameters
    ParameterUnary* density_ptr;

    // output file
    int num_ts_output = 0;
    std::string file_out_base_str;

    // functions
    void update(int ts, double dt);
    void set_output(int num_ts_output_in, std::string file_out_str);

    // default constructor
    PositionVelocitySphere() {}

    // constructor
    PositionVelocitySphere(SphereGroup &spheregroup_in, ParameterUnary &density_in)
    {

        // store variables
        spheregroup_ptr = &spheregroup_in;
        density_ptr = &density_in;

    }

    private:

    // functions
    void output_csv(int ts);

};

void PositionVelocitySphere::update(int ts, double dt)
{

    // write output file
    if (ts % num_ts_output == 0)
    {
        output_csv(ts);
    }

    // iterate through each sphere
    for (auto &sphere : spheregroup_ptr->sphere_vec)
    {
        
        // get properties
        int mid = sphere.mid;
        double radius = sphere.radius;
        double density = density_ptr->get_value(mid);

        // calculate masses
        double mass = (4./3.)*M_PI*density*radius*radius*radius;
        double moi = 0.4*mass*radius*radius;
        
        // calculate linear position and velocity
        sphere.velocity += sphere.force*dt/mass;
        sphere.position += sphere.velocity*dt;

        // calculate rotational position and velocity
        sphere.angularvelocity += sphere.moment*dt/moi;
        sphere.angularposition += sphere.angularvelocity*dt;

    }

}

void PositionVelocitySphere::set_output(int num_ts_output_in, std::string file_out_str)
{

    // set output parameters
    num_ts_output = num_ts_output_in;
    file_out_base_str = file_out_str;

}

void PositionVelocitySphere::output_csv(int ts)
{

    // do not make file if filename not set
    if (file_out_base_str.empty())
    {
        return;
    }

    // split filename at '*'
    // will be replaced with timestep later
    std::vector<std::string> file_out_base_vec;
    std::stringstream file_out_base_stream(file_out_base_str);
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
    for (auto &sphere : spheregroup_ptr->sphere_vec)
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
