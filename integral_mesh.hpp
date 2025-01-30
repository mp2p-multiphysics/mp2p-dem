#ifndef INTEGRAL_MESH
#define INTEGRAL_MESH
#include <fstream>
#include <sstream>
#include "container_typedef.hpp"
#include "group_mesh.hpp"
#include "integral_base.hpp"

namespace DEM
{

class IntegralMesh : public IntegralBase
{
    /*

    Updates the position of a moving mesh.

    Variables
    =========
    meshgroup_in : MeshGroup
        Meshes whose positions are updated.

    */

    public:

    // mesh group
    double dt = 0.;
    MeshGroup* meshgroup_ptr;

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    IntegralMesh() {}

    // constructor
    IntegralMesh(MeshGroup &meshgroup_in)
    {

        // store variables
        meshgroup_ptr = &meshgroup_in;

    }

    private:

};

void IntegralMesh::update(int ts)
{
    /*

    Updates this object.

    Arguments
    =========
    ts : int
        Timestep number.

    Returns
    =======
    (none)

    */

    // get mesh velocity
    EigenVector3D vel_trn = meshgroup_ptr->velocity_translate;
    double angvel_rot = meshgroup_ptr->angularvelocity_rotate;
    EigenVector3D rotaxis_p0 = meshgroup_ptr->position_rotateaxis_begin;
    EigenVector3D rotaxis_p1 = meshgroup_ptr->position_rotateaxis_end;

    // check if translation or rotation occurs
    // skip expensive calculations if it does not
    bool is_trn = vel_trn == EigenVector3D::Zero();
    bool is_rot = angvel_rot != 0. || rotaxis_p0 == rotaxis_p1;

    // skip if no motion
    if (is_trn && is_rot)
    {
        return;
    }

    // compute rotation matrix if rotation occurs
    Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
    if (is_rot)
    {
        
        // get normalized axis of rotation
        EigenVector3D rotaxis = rotaxis_p1 - rotaxis_p0;
        EigenVector3D rotaxis_unit = rotaxis/rotaxis.norm();

        // calculate differential rotation angle
        double delta_angpos_rot = angvel_rot*dt;
        
        // calculate rotation matrix elements
        Eigen::AngleAxisd eigen_rot(delta_angpos_rot, rotaxis_unit);
        rot_mat = eigen_rot.toRotationMatrix();

    }

    // iterate through each mesh triangle
    for (auto &mesh : meshgroup_ptr->mesh_vec)
    {

        // store initial position
        EigenVector3D pos_p0 = mesh.position_p0;
        EigenVector3D pos_p1 = mesh.position_p1;
        EigenVector3D pos_p2 = mesh.position_p2;

        // apply translation
        mesh.position_p0 += vel_trn*dt;
        mesh.position_p1 += vel_trn*dt;
        mesh.position_p2 += vel_trn*dt;

        // apply rotation
        if (is_rot)
        {

            // translate the mesh so that rotaxis_p0 is at the origin
            mesh.position_p0 -= rotaxis_p0;
            mesh.position_p1 -= rotaxis_p0;
            mesh.position_p2 -= rotaxis_p0;

            // apply the rotation
            mesh.position_p0 = rot_mat*mesh.position_p0;
            mesh.position_p1 = rot_mat*mesh.position_p1;
            mesh.position_p2 = rot_mat*mesh.position_p2;

            // undo translation of rotaxis_p0 to the origin
            mesh.position_p0 += rotaxis_p0;
            mesh.position_p1 += rotaxis_p0;
            mesh.position_p2 += rotaxis_p0;

        }

        // reset forces and moments
        meshgroup_ptr->force = {0., 0., 0.,};
        meshgroup_ptr->moment = {0., 0., 0.,};

    }

}

}

#endif
