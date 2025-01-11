#ifndef WALL_MESH
#define WALL_MESH
#include "container_typedef.hpp"

namespace DEM
{

class WallMesh
{

    public:

    // number of mesh elements
    int num_element = 0;

    // id of mesh triangles
    // assume that mesh is inserted or deleted as a whole
    // therefore permanent ID = temporary ID

    // materials
    int material = 0;

    // mesh point positions
    std::vector<EigenVector3D> position_p1_vec;
    std::vector<EigenVector3D> position_p2_vec;
    std::vector<EigenVector3D> position_p3_vec;

    // mesh velocities
    EigenVector3D velocity_vec;
    EigenVector3D angularvelocity_vec;
    EigenVector3D rotateaxis_p1_vec;
    EigenVector3D rotateaxis_p2_vec;

    // default constructor
    WallMesh() {}

    private:

};

}

#endif
