#ifndef CONTAINER_WALLMESH
#define CONTAINER_WALLMESH
#include <vector>

struct WallMeshPositionVelocityStruct
{
    /*
    
    Stores position and velocity of mesh triangles in a wall.

    Variables
    =========
    num_mesh : int
        Number of mesh triangles.
    num_mesh_historical_max : int
        Highest recorded number of mesh triangles in the simulation.
    id_vec : int
        ID of each mesh triangle
    type_vec : int
        Type of each mesh triangle.
    position_p1_x_vec : VectorDouble
        x-component of the position of the 1st point of each mesh triangle.
    position_p1_y_vec : VectorDouble
        y-component of the position of the 1st point of each mesh triangle.
    position_p1_z_vec : VectorDouble
        z-component of the position of the 1st point of each mesh triangle.
    position_p2_x_vec : VectorDouble
        x-component of the position of the 2nd point of each mesh triangle.
    position_p2_y_vec : VectorDouble
        y-component of the position of the 2nd point of each mesh triangle.
    position_p2_z_vec : VectorDouble
        z-component of the position of the 2nd point of each mesh triangle.
    position_p3_x_vec : VectorDouble
        x-component of the position of the 3rd point of each mesh triangle.
    position_p3_y_vec : VectorDouble
        y-component of the position of the 3rd point of each mesh triangle.
    position_p3_z_vec : VectorDouble
        z-component of the position of the 3rd point of each mesh triangle.
    velocity_translate_x : double
        x-component of the translational velocity of the wall.
    velocity_translate_y : double
        y-component of the translational velocity of the wall.
    velocity_translate_z : double
        z-component of the translational velocity of the wall.
    angularvelocity_rotate : double
        Angular velocity of the wall.
    position_rotationaxis_p1_x : double
        x-component of the start point of the axis of rotation.
    position_rotationaxis_p1_y : double
        y-component of the start point of the axis of rotation.
    position_rotationaxis_p1_z : double
        z-component of the start point of the axis of rotation.
    position_rotationaxis_p2_x : double
        x-component of the end point of the axis of rotation.
    position_rotationaxis_p2_y : double
        y-component of the end point of the axis of rotation.
    position_rotationaxis_p2_z : double
        z-component of the end point of the axis of rotation.

    Notes
    =====
    The angular velocity vector points in the same direction as the axis of rotation.

    */

    // number of mesh triangles
    int num_mesh = 0;
    int num_mesh_historical_max = 0;

    // id and type of mesh triangles
    std::vector<int> id_vec;
    std::vector<int> type_vec;

    // positions
    std::vector<double> position_p1_x_vec;
    std::vector<double> position_p1_y_vec;
    std::vector<double> position_p1_z_vec;
    std::vector<double> position_p2_x_vec;
    std::vector<double> position_p2_y_vec;
    std::vector<double> position_p2_z_vec;
    std::vector<double> position_p3_x_vec;
    std::vector<double> position_p3_y_vec;
    std::vector<double> position_p3_z_vec;

    // translational velocities
    double velocity_translate_x = 0.;
    double velocity_translate_y = 0.;
    double velocity_translate_z = 0.;

    // rotational velocities
    double angularvelocity_rotate = 0.;
    double position_rotateaxis_p1_x = 0.;
    double position_rotateaxis_p1_y = 0.;
    double position_rotateaxis_p1_z = 0.;
    double position_rotateaxis_p2_x = 0.;
    double position_rotateaxis_p2_y = 0.;
    double position_rotateaxis_p2_z = 0.;

};

void wallmesh_set_velocity_translate
(
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    double velocity_translate_x, double velocity_translate_y, double velocity_translate_z
)
{
    /*
    
    Sets the translational velocity of a wall.

    Arguments
    =========
    wallmesh_pvs : WallMeshPositionVelocityStruct
        Positions and velocities of the mesh triangles in a wall.
    velocity_translate_x : double
        x-component of the translational velocity of the wall.
    velocity_translate_y : double
        y-component of the translational velocity of the wall.
    velocity_translate_z : double
        z-component of the translational velocity of the wall.

    Returns
    =======
    (none)

    */
    
    // set translational velocities
    wallmesh_pvs.velocity_translate_x = velocity_translate_x;
    wallmesh_pvs.velocity_translate_y = velocity_translate_y;
    wallmesh_pvs.velocity_translate_z = velocity_translate_z;

}

void wallmesh_set_angularvelocity_rotate
(
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    double angularvelocity_rotate,
    double position_rotateaxis_p1_x, double position_rotateaxis_p1_y, double position_rotateaxis_p1_z,
    double position_rotateaxis_p2_x, double position_rotateaxis_p2_y, double position_rotateaxis_p2_z
)
{
    /*
    
    Sets the rotational velocity of a wall.

    Arguments
    =========
    wallmesh_pvs : WallMeshPositionVelocityStruct
        Positions and velocities of the mesh triangles in a wall.
    angularvelocity_rotate : double
        Angular velocity of the wall.
    position_rotationaxis_p1_x : double
        x-coordinate of the start point of the axis of rotation.
    position_rotationaxis_p1_y : double
        y-coordinate of the start point of the axis of rotation.
    position_rotationaxis_p1_z : double
        z-coordinate of the start point of the axis of rotation.
    position_rotationaxis_p2_x : double
        x-coordinate of the end point of the axis of rotation.
    position_rotationaxis_p2_y : double
        y-coordinate of the end point of the axis of rotation.
    position_rotationaxis_p2_z : double
        z-coordinate of the end point of the axis of rotation.

    Returns
    =======
    (none)

    */
    
    // set rotational velocities
    wallmesh_pvs.angularvelocity_rotate = angularvelocity_rotate;
    wallmesh_pvs.position_rotateaxis_p1_x = position_rotateaxis_p1_x;
    wallmesh_pvs.position_rotateaxis_p1_y = position_rotateaxis_p1_y;
    wallmesh_pvs.position_rotateaxis_p1_z = position_rotateaxis_p1_z;
    wallmesh_pvs.position_rotateaxis_p2_x = position_rotateaxis_p2_x;
    wallmesh_pvs.position_rotateaxis_p2_y = position_rotateaxis_p2_y;
    wallmesh_pvs.position_rotateaxis_p2_z = position_rotateaxis_p2_z;

}

#endif
