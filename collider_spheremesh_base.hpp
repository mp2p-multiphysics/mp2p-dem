#ifndef COLLIDER_SPHEREMESH_BASE
#define COLLIDER_SPHEREMESH_BASE
#include "container_typedef.hpp"

namespace DEM
{

class ColliderSphereMeshBase
{

    public:

    // functions
    virtual std::vector<std::pair<int, int>> get_collision_vec() {return {};};
    virtual void update_collision_vec() {};

    // default constructor
    ColliderSphereMeshBase() {}

    private:

};

}

#endif
