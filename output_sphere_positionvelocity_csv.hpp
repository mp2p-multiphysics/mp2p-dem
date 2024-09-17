#ifndef OUTPUT_SPHERE_POSITIONVELOCITY_CSV
#define OUTPUT_SPHERE_POSITIONVELOCITY_CSV
#include <fstream>
#include <sstream>
#include <vector>
#include "container_sphere.hpp"

class OutputSpherePositionVelocityCSV
{
    /*

    Outputs the positions and velocities of each sphere to a CSV file.

    Variables
    =========
    file_out_base_str_in : string
        Base file name of the output files.

    Functions
    =========
    output_positionvelocity : void
        Outputs the positions and velocities of each sphere to a CSV.

    Notes
    =====
    The base file name should contain an asterisk '*', which will be replaced with the time step.

    */

    public:

    // variables
    std::string file_out_base_str;
    std::vector<std::string> file_out_base_vec;

    // functions
    void output_positionvelocity(SpherePositionVelocityStruct &sphere_pvs, int ts);

    // default constructor
    OutputSpherePositionVelocityCSV()
    {

    }

    // constructor
    OutputSpherePositionVelocityCSV(std::string file_out_base_str_in)
    {

        // store variables
        file_out_base_str = file_out_base_str_in;

        // split filename at '*'
        // will be replaced with timestep later
        std::stringstream file_out_base_stream(file_out_base_str);
        std::string string_sub;
        while(std::getline(file_out_base_stream, string_sub, '*'))
        {
            file_out_base_vec.push_back(string_sub);
        }

    }

};

void OutputSpherePositionVelocityCSV::output_positionvelocity(SpherePositionVelocityStruct &sphere_pvs, int ts)
{
    /*

    Outputs the net force and moment acting on each sphere to a CSV.

    Arguments
    =========
    sphere_pvs : SpherePositionVelocityStruct
        struct with position and velocity of each sphere.
    ts : int
        nth timestep in the simulation.

    Returns
    =======
    (none)

    */

    // create output filename
    // replace '*' with timestep
    std::string file_out_str = file_out_base_vec[0];
    for (int indx_i = 1; indx_i < file_out_base_vec.size(); indx_i++)
    {
        file_out_str += std::to_string(ts) + file_out_base_vec[indx_i];
    }

    // initialize output file
    std::ofstream file_out_stream(file_out_str);

    // write output data
    file_out_stream << "id,type,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,angpos_x,angpos_y,angpos_z,angvel_x,angvel_y,angvel_z\n";
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++)
    {
        file_out_stream << sphere_pvs.id_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.type_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.position_x_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.position_y_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.position_z_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.velocity_x_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.velocity_y_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.velocity_z_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.angularposition_x_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.angularposition_y_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.angularposition_z_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.angularvelocity_x_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.angularvelocity_y_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.angularvelocity_z_vec[indx_i] << "\n";
    }

    // close
    file_out_stream.close();

}

#endif
