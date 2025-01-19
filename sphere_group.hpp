#ifndef SPHERE_GROUP
#define SPHERE_GROUP
#include "container_typedef.hpp"
#include "sphere.hpp"

namespace DEM
{

class SphereGroup
{

    public:

    // largest inserted ID so far
    int gid_max = 0;

    // vector of spheres
    std::vector<Sphere> sphere_vec;

    // default constructor
    SphereGroup() {}

    private:

};

}

#endif
