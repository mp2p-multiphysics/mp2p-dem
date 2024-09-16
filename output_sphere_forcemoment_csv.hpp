#ifndef OUTPUT_SPHERE_FORCEMOMENT_CSV
#define OUTPUT_SPHERE_FORCEMOMENT_CSV
#include <fstream>
#include <sstream>
#include <vector>
#include "container_sphere.hpp"

class OutputSphereForceMomentCSV
{

    public:

    // variables
    std::string file_out_base_str;
    std::vector<std::string> file_out_base_vec;

    // functions
    void output_forcemoment(SpherePositionVelocityStruct &sphere_pvs, SphereForceMomentStruct &sphere_fms, int ts);

    // default constructor
    OutputSphereForceMomentCSV()
    {

    }

    // constructor
    OutputSphereForceMomentCSV(std::string file_out_base_str_in)
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

void OutputSphereForceMomentCSV::output_forcemoment(SpherePositionVelocityStruct &sphere_pvs, SphereForceMomentStruct &sphere_fms, int ts)
{

    // create output filename
    // replace '*' with timestep
    std::string file_out_str = file_out_base_vec[0];
    for (int i = 1; i < file_out_base_vec.size(); i++)
    {
        file_out_str += std::to_string(ts) + file_out_base_vec[i];
    }

    // initialize output file
    std::ofstream file_out_stream(file_out_str);

    // write output data
    file_out_stream << "id,type,fce_x,fce_y,fce_z,mom_x,mom_y,mom_z\n";
    for (int indx_i = 0; indx_i < sphere_fms.num_particle; indx_i++)
    {

        // calculate number of contacts for averaging
        int num_contact = sphere_fms.num_contact_vec[indx_i];
        double inv_num_contact = 0.;
        if (num_contact != 0)
        {
            inv_num_contact = 1./(double) num_contact;
        }

        // calculate net forces and moments on particle i
        double frc_x_i = sphere_fms.force_sum_x_vec[indx_i] + inv_num_contact*sphere_fms.force_average_x_vec[indx_i];
        double frc_y_i = sphere_fms.force_sum_y_vec[indx_i] + inv_num_contact*sphere_fms.force_average_y_vec[indx_i];
        double frc_z_i = sphere_fms.force_sum_z_vec[indx_i] + inv_num_contact*sphere_fms.force_average_z_vec[indx_i];
        double mom_x_i = sphere_fms.moment_sum_x_vec[indx_i] + inv_num_contact*sphere_fms.moment_average_x_vec[indx_i];
        double mom_y_i = sphere_fms.moment_sum_y_vec[indx_i] + inv_num_contact*sphere_fms.moment_average_y_vec[indx_i];
        double mom_z_i = sphere_fms.moment_sum_z_vec[indx_i] + inv_num_contact*sphere_fms.moment_average_z_vec[indx_i];

        // write to csv
        file_out_stream << sphere_pvs.id_vec[indx_i] << ",";
        file_out_stream << sphere_pvs.type_vec[indx_i] << ",";
        file_out_stream << frc_x_i << ",";
        file_out_stream << frc_y_i << ",";
        file_out_stream << frc_z_i << ",";
        file_out_stream << mom_x_i << ",";
        file_out_stream << mom_y_i << ",";
        file_out_stream << mom_z_i << "\n";

    }

    // close
    file_out_stream.close();

}

#endif
