#ifndef OUTPUT_SPHERE_CSV
#define OUTPUT_SPHERE_CSV
#include <fstream>
#include <vector>
#include "container_sphere.hpp"

void output_sphere_csv(SpherePositionVelocityStruct sphere_pvs, std::string file_out_base_str, int ts)
{

    // initialize output file
    std::string file_out_str = file_out_base_str + std::to_string(ts) + ".csv";
    std::ofstream file_out_stream(file_out_str);

    // write output data
    file_out_stream << "id,type,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,angpos_x,angpos_y,angpos_z,angvel_x,angvel_y,angvel_z\n";
    for (int i = 0; i < sphere_pvs.num_particle; i++)
    {
        file_out_stream << sphere_pvs.id_vec[i] << ",";
        file_out_stream << sphere_pvs.type_vec[i] << ",";
        file_out_stream << sphere_pvs.position_x_vec[i] << ",";
        file_out_stream << sphere_pvs.position_y_vec[i] << ",";
        file_out_stream << sphere_pvs.position_z_vec[i] << ",";
        file_out_stream << sphere_pvs.velocity_x_vec[i] << ",";
        file_out_stream << sphere_pvs.velocity_y_vec[i] << ",";
        file_out_stream << sphere_pvs.velocity_z_vec[i] << ",";
        file_out_stream << sphere_pvs.angularposition_x_vec[i] << ",";
        file_out_stream << sphere_pvs.angularposition_y_vec[i] << ",";
        file_out_stream << sphere_pvs.angularposition_z_vec[i] << ",";
        file_out_stream << sphere_pvs.angularvelocity_x_vec[i] << ",";
        file_out_stream << sphere_pvs.angularvelocity_y_vec[i] << ",";
        file_out_stream << sphere_pvs.angularvelocity_z_vec[i] << "\n";
    }

    // close
    file_out_stream.close();

}

#endif
