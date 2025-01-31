#ifndef DELETE_MESH
#define DELETE_MESH
#include "insertdelete_base.hpp"
#include "group_mesh.hpp"

namespace DEM
{

class DeleteMesh : public InsertDeleteBase
{
    /*

    Deletes a mesh at a specified time.

    Variables
    =========
    meshgroup_in : MeshGroup
        Mesh group to be deleted.
    ts_delete_in : int
        Timestep at which the mesh is deleted.

    */

    public:

    // memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // mesh group
    double dt = 0.;
    MeshGroup* meshgroup_ptr;

    // timesteps
    int ts_delete = 0;

    // functions
    void initialize(double dt_in) {dt = dt_in;};
    void update(int ts);

    // default constructor
    DeleteMesh() {}

    // constructor
    DeleteMesh(MeshGroup &meshgroup_in, int ts_delete_in)
    {

        // store inputs
        meshgroup_ptr = &meshgroup_in;
        ts_delete = ts_delete_in;

    }

    private:

};

void DeleteMesh::update(int ts)
{
    /*

    Updates this object.

    Arguments
    =========
    ts : int
        Timestep number.
    
    Returns
    =======
    (none)

    */

    // check if insertion is needed
    if (ts != ts_delete)
    {
        return;  
    }

    // reset mesh group
    meshgroup_ptr->num_mesh = 0;
    meshgroup_ptr->mesh_vec.clear();
    meshgroup_ptr->point_vec.clear();
    meshgroup_ptr->mid = 0;
    meshgroup_ptr->velocity_translate = {0., 0., 0.};
    meshgroup_ptr->angularvelocity_rotate = 0.;
    meshgroup_ptr->position_rotateaxis_begin = {0., 0., 0.};
    meshgroup_ptr->position_rotateaxis_end = {0., 0., 1.};

}

}

#endif
