#ifndef OUTPUT_WALLMESH_POSITION_STL
#define OUTPUT_WALLMESH_POSITION_STL
#include <fstream>
#include <sstream>
#include <vector>
#include "container_wallmesh.hpp"

class OutputWallMeshPositionSTL
{
    /*

    Outputs the position of the wall to a STL file.

    Variables
    =========
    file_out_base_str_in : string
        Base file name of the output files.

    Functions
    =========
    output_positionvelocity : void
        Outputs the position of the wall to a STL file.

    Notes
    =====
    The base file name should contain an asterisk '*', which will be replaced with the time step.

    */

    public:

    // variables
    std::string file_out_base_str;
    std::vector<std::string> file_out_base_vec;

    // functions
    void output_position(WallMeshPositionVelocityStruct &wallmesh_pvs, int ts);

    // default constructor
    OutputWallMeshPositionSTL ()
    {

    }

    // constructor
    OutputWallMeshPositionSTL (std::string file_out_base_str_in)
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

void OutputWallMeshPositionSTL::output_position(WallMeshPositionVelocityStruct &wallmesh_pvs, int ts)
{
    /*

    Outputs the position of the wall to a STL file.

    Arguments
    =========
    wallmesh_pvs : WallMeshPositionVelocityStruct
        struct with position and velocity of mesh triangles in a wall.
    ts : int
        nth timestep in the simulation.

    Returns
    =======
    (none)

    */

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
        double front_x_ki =  (pos_p1_y_k - pos_p2_y_k)*(pos_p1_z_k - pos_p3_z_k) - (pos_p1_y_k - pos_p3_y_k)*(pos_p1_z_k - pos_p2_z_k);
        double front_y_ki = -(pos_p1_x_k - pos_p2_x_k)*(pos_p1_z_k - pos_p3_z_k) + (pos_p1_x_k - pos_p3_x_k)*(pos_p1_z_k - pos_p2_z_k);
        double front_z_ki =  (pos_p1_x_k - pos_p2_x_k)*(pos_p1_y_k - pos_p3_y_k) - (pos_p1_x_k - pos_p3_x_k)*(pos_p1_y_k - pos_p2_y_k);

        // calculate front facing normal vector
        double helpvar_01 = 1./sqrt(front_x_ki*front_x_ki + front_y_ki*front_y_ki + front_z_ki*front_z_ki);
        double norm_front_x_ki = front_x_ki*helpvar_01;
        double norm_front_y_ki = front_y_ki*helpvar_01;
        double norm_front_z_ki = front_z_ki*helpvar_01;

        // write lines in file
        file_out_stream << "facet normal " << norm_front_x_ki << " " << norm_front_y_ki  << " " << norm_front_z_ki << "\n";
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
