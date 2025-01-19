#ifndef MESH
#define MESH
#include "container_typedef.hpp"

namespace DEM
{

struct Mesh
{

    // group ID
    int gid;

    // positions
    EigenVector3D position_p0;
    EigenVector3D position_p1;
    EigenVector3D position_p2;

    // velocity
    EigenVector3D velocity_translate;
    double angularvelocity_rotate;
    EigenVector3D position_rotateaxis_begin;
    EigenVector3D position_rotateaxis_end;

    // enlarged geometry for collision detection
    EigenVector3D position_min_enlarged;
    EigenVector3D position_max_enlarged;

};

}

#endif
