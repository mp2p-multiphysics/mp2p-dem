#ifndef SPHERE
#define SPHERE
#include "container_typedef.hpp"

namespace DEM
{

struct Sphere
{

    // group ID
    int gid;

    // material ID
    int mid;

    // positions and velocities
    EigenVector3D position;
    EigenVector3D velocity;
    EigenVector3D angularposition;
    EigenVector3D angularvelocity;

    // forces and moments
    EigenVector3D force;
    EigenVector3D moment;

    // geometry
    double radius;

    // enlarged geometry for collision detection
    double radius_enlarged;

};

}

#endif
