#ifndef CONTAINER_WALLMESH
#define CONTAINER_WALLMESH
#include <vector>

struct WallMeshPositionVelocityStruct
{
 
    int num_mesh = 0;
    int num_mesh_historical_max = 0;

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
    double angularvelocity_rotate = 0.;
    double position_rotateaxis_p1_x = 0.;
    double position_rotateaxis_p1_y = 0.;
    double position_rotateaxis_p1_z = 0.;
    double position_rotateaxis_p2_x = 0.;
    double position_rotateaxis_p2_y = 0.;
    double position_rotateaxis_p2_z = 0.;

};

void wallmesh_set_velocity_translate(
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    double velocity_translate_x, double velocity_translate_y, double velocity_translate_z
)
{
    wallmesh_pvs.velocity_translate_x = velocity_translate_x;
    wallmesh_pvs.velocity_translate_y = velocity_translate_y;
    wallmesh_pvs.velocity_translate_z = velocity_translate_z;
}

void wallmesh_set_angularvelocity_rotate(
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    double angularvelocity_rotate,
    double position_rotateaxis_p1_x, double position_rotateaxis_p1_y, double position_rotateaxis_p1_z,
    double position_rotateaxis_p2_x, double position_rotateaxis_p2_y, double position_rotateaxis_p2_z
)
{
    wallmesh_pvs.angularvelocity_rotate = angularvelocity_rotate;
    wallmesh_pvs.position_rotateaxis_p1_x = position_rotateaxis_p1_x;
    wallmesh_pvs.position_rotateaxis_p1_y = position_rotateaxis_p1_y;
    wallmesh_pvs.position_rotateaxis_p1_z = position_rotateaxis_p1_z;
    wallmesh_pvs.position_rotateaxis_p2_x = position_rotateaxis_p2_x;
    wallmesh_pvs.position_rotateaxis_p2_y = position_rotateaxis_p2_y;
    wallmesh_pvs.position_rotateaxis_p2_z = position_rotateaxis_p2_z;
}

#endif
