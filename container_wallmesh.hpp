#ifndef CONTAINER_WALLMESH
#define CONTAINER_WALLMESH
#include <vector>

struct WallMeshPositionVelocityStruct
{
 
    int num_mesh = 0;
    std::vector<int> id_vec;
    std::vector<int> type_vec;
    std::vector<double> position_p1_x_vec;
    std::vector<double> position_p1_y_vec;
    std::vector<double> position_p1_z_vec;
    std::vector<double> position_p2_x_vec;
    std::vector<double> position_p2_y_vec;
    std::vector<double> position_p2_z_vec;
    std::vector<double> position_p3_x_vec;
    std::vector<double> position_p3_y_vec;
    std::vector<double> position_p3_z_vec;
    double velocity_translate_x = 0.;
    double velocity_translate_y = 0.;
    double velocity_translate_z = 0.;
    double axis_rotate_p1_x = 0.;
    double axis_rotate_p1_y = 0.;
    double axis_rotate_p1_z = 0.;
    double axis_rotate_p2_x = 0.;
    double axis_rotate_p2_y = 0.;
    double axis_rotate_p2_z = 0.;
    double angularvelocity_rotate = 0.;

};

#endif
