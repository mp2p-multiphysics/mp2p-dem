#ifndef INTEGRATE_WALLMESH_EULER
#define INTEGRATE_WALLMESH_EULER
#include <vector>
#include "container_typedef.hpp"
#include "container_wallmesh.hpp"

class IntegrateWallMeshEuler
{

    public:

    // variables
    double dt;

    // functions
    void integrate_positionvelocity(WallMeshPositionVelocityStruct &wallmesh_pvs);

    // default constructor
    IntegrateWallMeshEuler()
    {

    }

    // constructor
    IntegrateWallMeshEuler(double dt_in)
    {
        dt = dt_in;
    }

};


void IntegrateWallMeshEuler::integrate_positionvelocity(WallMeshPositionVelocityStruct &wallmesh_pvs)
{

    // get wall velocities
    double vel_translate_x = wallmesh_pvs.velocity_translate_x;
    double vel_translate_y = wallmesh_pvs.velocity_translate_y;
    double vel_translate_z = wallmesh_pvs.velocity_translate_z;
    double angvel_rotate = wallmesh_pvs.angularvelocity_rotate;

    // skip if translation and rotation are zero
    if (vel_translate_x == 0. && vel_translate_y == 0. && vel_translate_z == 0. && angvel_rotate == 0.)
    {
        return;
    }

    // iterate for each mesh triangle
    for (int iter_k = 0; iter_k < wallmesh_pvs.num_mesh; iter_k++)
    {
        
        // translate points using velocity
        wallmesh_pvs.position_p1_x_vec[iter_k] += vel_translate_x*dt;
        wallmesh_pvs.position_p1_y_vec[iter_k] += vel_translate_y*dt;
        wallmesh_pvs.position_p1_z_vec[iter_k] += vel_translate_z*dt;
        wallmesh_pvs.position_p2_x_vec[iter_k] += vel_translate_x*dt;
        wallmesh_pvs.position_p2_y_vec[iter_k] += vel_translate_y*dt;
        wallmesh_pvs.position_p2_z_vec[iter_k] += vel_translate_z*dt;
        wallmesh_pvs.position_p3_x_vec[iter_k] += vel_translate_x*dt;
        wallmesh_pvs.position_p3_y_vec[iter_k] += vel_translate_y*dt;
        wallmesh_pvs.position_p3_z_vec[iter_k] += vel_translate_z*dt;

    }

    // skip if rotation is zero
    if (angvel_rotate == 0.)
    {
        return;
    }

    // get rotation axis
    double pos_rotateaxis_p1_x = wallmesh_pvs.position_rotateaxis_p1_x;
    double pos_rotateaxis_p1_y = wallmesh_pvs.position_rotateaxis_p1_y;
    double pos_rotateaxis_p1_z = wallmesh_pvs.position_rotateaxis_p1_z;
    double pos_rotateaxis_p2_x = wallmesh_pvs.position_rotateaxis_p2_x;
    double pos_rotateaxis_p2_y = wallmesh_pvs.position_rotateaxis_p2_y;
    double pos_rotateaxis_p2_z = wallmesh_pvs.position_rotateaxis_p2_z;

    // axis of rotation is specified using two points: axis P1 and axis P2
    // calculate axis P2 if axis P1 is moved to the origin
    double rotateaxis_x = -pos_rotateaxis_p1_x + pos_rotateaxis_p2_x;
    double rotateaxis_y = -pos_rotateaxis_p1_y + pos_rotateaxis_p2_y;
    double rotateaxis_z = -pos_rotateaxis_p1_z + pos_rotateaxis_p2_z;

    // normalize vector along axis of rotation
    double helpvar_01 = 1./sqrt(rotateaxis_x*rotateaxis_x + rotateaxis_y*rotateaxis_y + rotateaxis_z*rotateaxis_z);
    double unit_rotateaxis_x = rotateaxis_x*helpvar_01;
    double unit_rotateaxis_y = rotateaxis_y*helpvar_01;
    double unit_rotateaxis_z = rotateaxis_z*helpvar_01;
    if (rotateaxis_x == 0. && rotateaxis_y == 0. && rotateaxis_z == 0.)
    {
        unit_rotateaxis_x = 0.;
        unit_rotateaxis_y = 0.;
        unit_rotateaxis_z = 0.;
    }

    // calculate differential rotation angle
    double delta_angpos_rotate = angvel_rotate*dt;

    // calculate rotation matrix elements
    double rotate_xx = unit_rotateaxis_x*unit_rotateaxis_x*(1 - cos(delta_angpos_rotate)) + cos(delta_angpos_rotate);
    double rotate_xy = unit_rotateaxis_x*unit_rotateaxis_y*(1 - cos(delta_angpos_rotate)) - unit_rotateaxis_z*sin(delta_angpos_rotate);
    double rotate_xz = unit_rotateaxis_x*unit_rotateaxis_z*(1 - cos(delta_angpos_rotate)) + unit_rotateaxis_y*sin(delta_angpos_rotate);
    double rotate_yx = unit_rotateaxis_x*unit_rotateaxis_y*(1 - cos(delta_angpos_rotate)) + unit_rotateaxis_z*sin(delta_angpos_rotate);
    double rotate_yy = unit_rotateaxis_y*unit_rotateaxis_y*(1 - cos(delta_angpos_rotate)) + cos(delta_angpos_rotate);
    double rotate_yz = -unit_rotateaxis_x*sin(delta_angpos_rotate) + unit_rotateaxis_y*unit_rotateaxis_z*(1 - cos(delta_angpos_rotate));
    double rotate_zx = unit_rotateaxis_x*unit_rotateaxis_z*(1 - cos(delta_angpos_rotate)) - unit_rotateaxis_y*sin(delta_angpos_rotate);
    double rotate_zy = unit_rotateaxis_x*sin(delta_angpos_rotate) + unit_rotateaxis_y*unit_rotateaxis_z*(1 - cos(delta_angpos_rotate));
    double rotate_zz = unit_rotateaxis_z*unit_rotateaxis_z*(1 - cos(delta_angpos_rotate)) + cos(delta_angpos_rotate);

    // iterate for each mesh triangle
    for (int iter_k = 0; iter_k < wallmesh_pvs.num_mesh; iter_k++)
    {

        // calculate triangle points if axis P1 is moved to the origin
        double delta_pos_p1_origin_x = -pos_rotateaxis_p1_x + wallmesh_pvs.position_p1_x_vec[iter_k];
        double delta_pos_p1_origin_y = -pos_rotateaxis_p1_y + wallmesh_pvs.position_p1_y_vec[iter_k];
        double delta_pos_p1_origin_z = -pos_rotateaxis_p1_z + wallmesh_pvs.position_p1_z_vec[iter_k];
        double delta_pos_p2_origin_x = -pos_rotateaxis_p1_x + wallmesh_pvs.position_p2_x_vec[iter_k];
        double delta_pos_p2_origin_y = -pos_rotateaxis_p1_y + wallmesh_pvs.position_p2_y_vec[iter_k];
        double delta_pos_p2_origin_z = -pos_rotateaxis_p1_z + wallmesh_pvs.position_p2_z_vec[iter_k];
        double delta_pos_p3_origin_x = -pos_rotateaxis_p1_x + wallmesh_pvs.position_p3_x_vec[iter_k];
        double delta_pos_p3_origin_y = -pos_rotateaxis_p1_y + wallmesh_pvs.position_p3_y_vec[iter_k];
        double delta_pos_p3_origin_z = -pos_rotateaxis_p1_z + wallmesh_pvs.position_p3_z_vec[iter_k];

        // rotate the point around the axis of rotation
        double delta_pos_p1_origin_x_rotate = delta_pos_p1_origin_x*rotate_xx + delta_pos_p1_origin_y*rotate_xy + delta_pos_p1_origin_z*rotate_xz;
        double delta_pos_p1_origin_y_rotate = delta_pos_p1_origin_x*rotate_yx + delta_pos_p1_origin_y*rotate_yy + delta_pos_p1_origin_z*rotate_yz;
        double delta_pos_p1_origin_z_rotate = delta_pos_p1_origin_x*rotate_zx + delta_pos_p1_origin_y*rotate_zy + delta_pos_p1_origin_z*rotate_zz;
        double delta_pos_p2_origin_x_rotate = delta_pos_p2_origin_x*rotate_xx + delta_pos_p2_origin_y*rotate_xy + delta_pos_p2_origin_z*rotate_xz;
        double delta_pos_p2_origin_y_rotate = delta_pos_p2_origin_x*rotate_yx + delta_pos_p2_origin_y*rotate_yy + delta_pos_p2_origin_z*rotate_yz;
        double delta_pos_p2_origin_z_rotate = delta_pos_p2_origin_x*rotate_zx + delta_pos_p2_origin_y*rotate_zy + delta_pos_p2_origin_z*rotate_zz;
        double delta_pos_p3_origin_x_rotate = delta_pos_p3_origin_x*rotate_xx + delta_pos_p3_origin_y*rotate_xy + delta_pos_p3_origin_z*rotate_xz;
        double delta_pos_p3_origin_y_rotate = delta_pos_p3_origin_x*rotate_yx + delta_pos_p3_origin_y*rotate_yy + delta_pos_p3_origin_z*rotate_yz;
        double delta_pos_p3_origin_z_rotate = delta_pos_p3_origin_x*rotate_zx + delta_pos_p3_origin_y*rotate_zy + delta_pos_p3_origin_z*rotate_zz;

        // undo the translation of axis P1
        wallmesh_pvs.position_p1_x_vec[iter_k] = pos_rotateaxis_p1_x + delta_pos_p1_origin_x_rotate;
        wallmesh_pvs.position_p1_y_vec[iter_k] = pos_rotateaxis_p1_y + delta_pos_p1_origin_y_rotate;
        wallmesh_pvs.position_p1_z_vec[iter_k] = pos_rotateaxis_p1_z + delta_pos_p1_origin_z_rotate;
        wallmesh_pvs.position_p2_x_vec[iter_k] = pos_rotateaxis_p1_x + delta_pos_p2_origin_x_rotate;
        wallmesh_pvs.position_p2_y_vec[iter_k] = pos_rotateaxis_p1_y + delta_pos_p2_origin_y_rotate;
        wallmesh_pvs.position_p2_z_vec[iter_k] = pos_rotateaxis_p1_z + delta_pos_p2_origin_z_rotate;
        wallmesh_pvs.position_p3_x_vec[iter_k] = pos_rotateaxis_p1_x + delta_pos_p3_origin_x_rotate;
        wallmesh_pvs.position_p3_y_vec[iter_k] = pos_rotateaxis_p1_y + delta_pos_p3_origin_y_rotate;
        wallmesh_pvs.position_p3_z_vec[iter_k] = pos_rotateaxis_p1_z + delta_pos_p3_origin_z_rotate;

    }

}

#endif
