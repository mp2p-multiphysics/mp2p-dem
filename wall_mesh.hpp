#ifndef WALL_MESH
#define WALL_MESH
#include "container_typedef.hpp"

namespace DEM
{

class WallMesh
{

    public:

    // number of mesh parts
    int num_face = 0;
    int num_edge = 0;
    int num_vertex = 0;

    // assume that mesh is inserted or deleted as a whole
    // no need to account for partial deletion

    // id of mesh parts
    // fid - face ID
    // eid - edge ID
    // pid - point/vertex ID; used to index mesh positions
    // lid - local ID (e.g., 0, 1, and 2 for faces)
    VectorInt2D fid_lid_to_pid_vec;
    VectorInt2D eid_lid_to_pid_vec;

    // materials
    int material = 0;

    // mesh positions
    std::vector<EigenVector3D> position_vec;

    // mesh velocities
    EigenVector3D velocity_vec;
    double angularvelocity_vec;
    EigenVector3D rotateaxis_p1_vec;
    EigenVector3D rotateaxis_p2_vec;

    // default constructor
    WallMesh() {}

    private:

};

}

#endif
