#ifndef COLLIDER_SPHERESPHERE_BASE
#define COLLIDER_SPHERESPHERE_BASE
#include "container_typedef.hpp"
#include "sphere.hpp"

namespace DEM
{

class ColliderSphereSphereBase
{

    public:

    // functions
    virtual std::vector<std::pair<Sphere, Sphere>> get_collision_vec() {return {};};
    virtual void update_collision_vec() {};

    // default constructor
    ColliderSphereSphereBase() {}

    private:

};

}

#endif
