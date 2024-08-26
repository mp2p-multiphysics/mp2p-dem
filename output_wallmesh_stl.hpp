#ifndef OUTPUT_WALLMESH_STL
#define OUTPUT_WALLMESH_STL
#include <fstream>
#include <vector>
#include "container_wallmesh.hpp"

void output_wallmesh_stl(WallMeshPositionVelocityStruct wallmesh_pvs, std::string file_out_base_str, int ts)
{

    // initialize output file
    std::string file_out_str = file_out_base_str + std::to_string(ts) + ".stl";
    std::ofstream file_out_stream(file_out_str);

    // write output data
    file_out_stream << "solid mp2p\n";
    for (int indx_k = 0; indx_k < wallmesh_pvs.num_mesh; indx_k++)
    {

        // get wall points
        double pos_p1_x_k = wallmesh_pvs.position_p1_x_vec[indx_k];
        double pos_p1_y_k = wallmesh_pvs.position_p1_y_vec[indx_k];
        double pos_p1_z_k = wallmesh_pvs.position_p1_z_vec[indx_k];
        double pos_p2_x_k = wallmesh_pvs.position_p2_x_vec[indx_k];
        double pos_p2_y_k = wallmesh_pvs.position_p2_y_vec[indx_k];
        double pos_p2_z_k = wallmesh_pvs.position_p2_z_vec[indx_k];
        double pos_p3_x_k = wallmesh_pvs.position_p3_x_vec[indx_k];
        double pos_p3_y_k = wallmesh_pvs.position_p3_y_vec[indx_k];
        double pos_p3_z_k = wallmesh_pvs.position_p3_z_vec[indx_k];

        // calculate vector facing the "front" of the triangle
        double frnt_x_ki = (pos_p1_y_k - pos_p2_y_k)*(pos_p1_z_k - pos_p3_z_k) - (pos_p1_y_k - pos_p3_y_k)*(pos_p1_z_k - pos_p2_z_k);
        double frnt_y_ki = -(pos_p1_x_k - pos_p2_x_k)*(pos_p1_z_k - pos_p3_z_k) + (pos_p1_x_k - pos_p3_x_k)*(pos_p1_z_k - pos_p2_z_k);
        double frnt_z_ki = (pos_p1_x_k - pos_p2_x_k)*(pos_p1_y_k - pos_p3_y_k) - (pos_p1_x_k - pos_p3_x_k)*(pos_p1_y_k - pos_p2_y_k);

        // calculate front facing normal vector
        double helpvar_01 = 1./sqrt(frnt_x_ki*frnt_x_ki + frnt_y_ki*frnt_y_ki + frnt_z_ki*frnt_z_ki);
        double norm_frnt_x_ki = frnt_x_ki*helpvar_01;
        double norm_frnt_y_ki = frnt_y_ki*helpvar_01;
        double norm_frnt_z_ki = frnt_z_ki*helpvar_01;

        // write lines in file
        file_out_stream << "facet normal " << norm_frnt_x_ki << " " << norm_frnt_y_ki  << " " << norm_frnt_z_ki << "\n";
        file_out_stream << "  outer loop\n";
        file_out_stream << "    vertex " << pos_p1_x_k << " " << pos_p1_y_k << " " << pos_p1_z_k << "\n";
        file_out_stream << "    vertex " << pos_p2_x_k << " " << pos_p2_y_k << " " << pos_p2_z_k << "\n";
        file_out_stream << "    vertex " << pos_p3_x_k << " " << pos_p3_y_k << " " << pos_p3_z_k << "\n";
        file_out_stream << "  endloop\n";
        file_out_stream << "endfacet\n";

    }
    file_out_stream << "endsolid mp2p\n";

    // close
    file_out_stream.close();

}

#endif
