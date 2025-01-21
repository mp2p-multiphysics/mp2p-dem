#ifndef GROUP_MESH
#define GROUP_MESH
#include "container_typedef.hpp"
#include "group_base.hpp"

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

class MeshGroup : public BaseGroup
{

    public:

    // largest inserted ID so far
    int gid_max = 0;

    // vector of meshes
    std::vector<Mesh> mesh_vec;

    // functions
    void write_output(int ts) {};
    void clear_forcemoment() {};

    // default constructor
    MeshGroup() {}

    private:

};

}

#endif
