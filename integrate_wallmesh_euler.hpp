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
    void update_position_velocity(WallMeshPositionVelocityStruct &wallmesh_pvs);

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


void IntegrateWallMeshEuler::update_position_velocity(WallMeshPositionVelocityStruct &wallmesh_pvs)
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
    double axis_rotate_p1_x = wallmesh_pvs.axis_rotate_p1_x;
    double axis_rotate_p1_y = wallmesh_pvs.axis_rotate_p1_y;
    double axis_rotate_p1_z = wallmesh_pvs.axis_rotate_p1_z;
    double axis_rotate_p2_x = wallmesh_pvs.axis_rotate_p2_x;
    double axis_rotate_p2_y = wallmesh_pvs.axis_rotate_p2_y;
    double axis_rotate_p2_z = wallmesh_pvs.axis_rotate_p2_z;

    // axis of rotation is specified using two points: axis P1 and axis P2
    // calculate axis P2 if axis P1 is moved to the origin
    double axis_rotate_p2_origin_x = -axis_rotate_p1_x + axis_rotate_p2_x;
    double axis_rotate_p2_origin_y = -axis_rotate_p1_y + axis_rotate_p2_y;
    double axis_rotate_p2_origin_z = -axis_rotate_p1_z + axis_rotate_p2_z;

    // normalize vector along axis of rotation
    double helpvar_01 = 1./sqrt(axis_rotate_p2_origin_x*axis_rotate_p2_origin_x + axis_rotate_p2_origin_y*axis_rotate_p2_origin_y + axis_rotate_p2_origin_z*axis_rotate_p2_origin_z);
    double unit_axis_rotate_x = axis_rotate_p2_origin_x*helpvar_01;
    double unit_axis_rotate_y = axis_rotate_p2_origin_y*helpvar_01;
    double unit_axis_rotate_z = axis_rotate_p2_origin_z*helpvar_01;
    if (axis_rotate_p2_origin_x == 0. && axis_rotate_p2_origin_y == 0. && axis_rotate_p2_origin_z == 0.)
    {
        unit_axis_rotate_x = 0.;
        unit_axis_rotate_y = 0.;
        unit_axis_rotate_z = 0.;
    }

    // calculate differential rotation angle
    double dangpos_rotate = angvel_rotate*dt;

    // calculate rotation matrix elements
    double rotate_xx = unit_axis_rotate_x*unit_axis_rotate_x*(1 - cos(dangpos_rotate)) + cos(dangpos_rotate);
    double rotate_xy = unit_axis_rotate_x*unit_axis_rotate_y*(1 - cos(dangpos_rotate)) - unit_axis_rotate_z*sin(dangpos_rotate);
    double rotate_xz = unit_axis_rotate_x*unit_axis_rotate_z*(1 - cos(dangpos_rotate)) + unit_axis_rotate_y*sin(dangpos_rotate);
    double rotate_yx = unit_axis_rotate_x*unit_axis_rotate_y*(1 - cos(dangpos_rotate)) + unit_axis_rotate_z*sin(dangpos_rotate);
    double rotate_yy = unit_axis_rotate_y*unit_axis_rotate_y*(1 - cos(dangpos_rotate)) + cos(dangpos_rotate);
    double rotate_yz = -unit_axis_rotate_x*sin(dangpos_rotate) + unit_axis_rotate_y*unit_axis_rotate_z*(1 - cos(dangpos_rotate));
    double rotate_zx = unit_axis_rotate_x*unit_axis_rotate_z*(1 - cos(dangpos_rotate)) - unit_axis_rotate_y*sin(dangpos_rotate);
    double rotate_zy = unit_axis_rotate_x*sin(dangpos_rotate) + unit_axis_rotate_y*unit_axis_rotate_z*(1 - cos(dangpos_rotate));
    double rotate_zz = unit_axis_rotate_z*unit_axis_rotate_z*(1 - cos(dangpos_rotate)) + cos(dangpos_rotate);

    // iterate for each mesh triangle
    for (int iter_k = 0; iter_k < wallmesh_pvs.num_mesh; iter_k++)
    {

        // calculate triangle points if axis P1 is moved to the origin
        double pos_p1_origin_x = -axis_rotate_p1_x + wallmesh_pvs.position_p1_x_vec[iter_k];
        double pos_p1_origin_y = -axis_rotate_p1_y + wallmesh_pvs.position_p1_y_vec[iter_k];
        double pos_p1_origin_z = -axis_rotate_p1_z + wallmesh_pvs.position_p1_z_vec[iter_k];
        double pos_p2_origin_x = -axis_rotate_p1_x + wallmesh_pvs.position_p2_x_vec[iter_k];
        double pos_p2_origin_y = -axis_rotate_p1_y + wallmesh_pvs.position_p2_y_vec[iter_k];
        double pos_p2_origin_z = -axis_rotate_p1_z + wallmesh_pvs.position_p2_z_vec[iter_k];
        double pos_p3_origin_x = -axis_rotate_p1_x + wallmesh_pvs.position_p3_x_vec[iter_k];
        double pos_p3_origin_y = -axis_rotate_p1_y + wallmesh_pvs.position_p3_y_vec[iter_k];
        double pos_p3_origin_z = -axis_rotate_p1_z + wallmesh_pvs.position_p3_z_vec[iter_k];

        // rotate the point around the axis of rotation
        double pos_rotate_p1_origin_x = pos_p1_origin_x*rotate_xx + pos_p1_origin_y*rotate_xy + pos_p1_origin_z*rotate_xz;
        double pos_rotate_p1_origin_y = pos_p1_origin_x*rotate_yx + pos_p1_origin_y*rotate_yy + pos_p1_origin_z*rotate_yz;
        double pos_rotate_p1_origin_z = pos_p1_origin_x*rotate_zx + pos_p1_origin_y*rotate_zy + pos_p1_origin_z*rotate_zz;
        double pos_rotate_p2_origin_x = pos_p2_origin_x*rotate_xx + pos_p2_origin_y*rotate_xy + pos_p2_origin_z*rotate_xz;
        double pos_rotate_p2_origin_y = pos_p2_origin_x*rotate_yx + pos_p2_origin_y*rotate_yy + pos_p2_origin_z*rotate_yz;
        double pos_rotate_p2_origin_z = pos_p2_origin_x*rotate_zx + pos_p2_origin_y*rotate_zy + pos_p2_origin_z*rotate_zz;
        double pos_rotate_p3_origin_x = pos_p3_origin_x*rotate_xx + pos_p3_origin_y*rotate_xy + pos_p3_origin_z*rotate_xz;
        double pos_rotate_p3_origin_y = pos_p3_origin_x*rotate_yx + pos_p3_origin_y*rotate_yy + pos_p3_origin_z*rotate_yz;
        double pos_rotate_p3_origin_z = pos_p3_origin_x*rotate_zx + pos_p3_origin_y*rotate_zy + pos_p3_origin_z*rotate_zz;

        // undo the translation of axis P1
        wallmesh_pvs.position_p1_x_vec[iter_k] = axis_rotate_p1_x + pos_rotate_p1_origin_x;
        wallmesh_pvs.position_p1_y_vec[iter_k] = axis_rotate_p1_y + pos_rotate_p1_origin_y;
        wallmesh_pvs.position_p1_z_vec[iter_k] = axis_rotate_p1_z + pos_rotate_p1_origin_z;
        wallmesh_pvs.position_p2_x_vec[iter_k] = axis_rotate_p1_x + pos_rotate_p2_origin_x;
        wallmesh_pvs.position_p2_y_vec[iter_k] = axis_rotate_p1_y + pos_rotate_p2_origin_y;
        wallmesh_pvs.position_p2_z_vec[iter_k] = axis_rotate_p1_z + pos_rotate_p2_origin_z;
        wallmesh_pvs.position_p3_x_vec[iter_k] = axis_rotate_p1_x + pos_rotate_p3_origin_x;
        wallmesh_pvs.position_p3_y_vec[iter_k] = axis_rotate_p1_y + pos_rotate_p3_origin_y;
        wallmesh_pvs.position_p3_z_vec[iter_k] = axis_rotate_p1_z + pos_rotate_p3_origin_z;

    }

}

#endif
