#ifndef MESH_GROUP
#define MESH_GROUP
#include "container_typedef.hpp"
#include "mesh.hpp"

namespace DEM
{

class MeshGroup
{

    public:

    // largest inserted ID so far
    int gid_max = 0;

    // vector of meshes
    std::vector<Mesh> mesh_vec;

    // default constructor
    MeshGroup() {}

    private:

};

}

#endif
