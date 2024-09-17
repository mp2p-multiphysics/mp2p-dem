#ifndef DELETE_WALLMESH
#define DELETE_WALLMESH
#include <vector>
#include "container_wallmesh.hpp"

class DeleteWallMesh
{

    public:

    // functions
    void delete_wallmesh(WallMeshPositionVelocityStruct &wallmesh_pvs);

    // default constructor
    DeleteWallMesh()
    {

    }
    
};

void DeleteWallMesh::delete_wallmesh(WallMeshPositionVelocityStruct &wallmesh_pvs)
{

    // clear all wallmesh vectors
    wallmesh_pvs.id_vec.clear();
    wallmesh_pvs.type_vec.clear();
    wallmesh_pvs.position_p1_x_vec.clear();
    wallmesh_pvs.position_p1_y_vec.clear();
    wallmesh_pvs.position_p1_z_vec.clear();
    wallmesh_pvs.position_p2_x_vec.clear();
    wallmesh_pvs.position_p2_y_vec.clear();
    wallmesh_pvs.position_p2_z_vec.clear();
    wallmesh_pvs.position_p3_x_vec.clear();
    wallmesh_pvs.position_p3_y_vec.clear();
    wallmesh_pvs.position_p3_z_vec.clear();

    // reset all wallmesh scalars
    wallmesh_pvs.num_mesh = 0;
    wallmesh_pvs.velocity_translate_x = 0.;
    wallmesh_pvs.velocity_translate_y = 0.;
    wallmesh_pvs.velocity_translate_z = 0.;
    wallmesh_pvs.angularvelocity_rotate = 0.;
    wallmesh_pvs.position_rotateaxis_p1_x = 0.;
    wallmesh_pvs.position_rotateaxis_p1_y = 0.;
    wallmesh_pvs.position_rotateaxis_p1_z = 0.;
    wallmesh_pvs.position_rotateaxis_p2_x = 0.;
    wallmesh_pvs.position_rotateaxis_p2_y = 0.;
    wallmesh_pvs.position_rotateaxis_p2_z = 0.;

}

#endif
