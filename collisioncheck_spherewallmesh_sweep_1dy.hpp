#ifndef COLLISIONCHECK_SPHEREWALLMESH_SWEEP_1DY
#define COLLISIONCHECK_SPHEREWALLMESH_SWEEP_1DY
#include <utility>
#include <vector>
#include "container_sphere.hpp"
#include "container_typedef.hpp"
#include "container_wallmesh.hpp"

class CollisionCheckSphereWallMeshSweep1Dy
{

    public:

    // variables
    VectorDouble radius_vec;
    double enlarge_factor;

    // functions
    VectorPairInt broad_search(SpherePositionVelocityStruct &sphere_pvs, WallMeshPositionVelocityStruct &wallmesh_pvs);

    // default constructor
    CollisionCheckSphereWallMeshSweep1Dy()
    {

    }

    // constructor
    CollisionCheckSphereWallMeshSweep1Dy(VectorDouble radius_vec_in, double enlarge_factor_in = 0.05)
    {
        radius_vec = radius_vec_in;
        enlarge_factor = enlarge_factor_in;
    }

    private:

    // bound rank to id vector
    // input (index): bound rank; output (value): bound id
    VectorInt bound_rank_id_b_vec;

    // number of particles in previous usage
    int num_particle_old = 0;
    int num_particle_historical_max_old = 0;

    // functions
    void sort_pair_insertion(VectorPairIntDouble &pair_vec);

};

VectorPairInt CollisionCheckSphereWallMeshSweep1Dy::broad_search(SpherePositionVelocityStruct &sphere_pvs, WallMeshPositionVelocityStruct &wallmesh_pvs)
{

    // sweep along y axis

    // initialize vector with lower and upper bound
    VectorDouble bound_y_vec;

    // calculate lower and upper bounds of particles
    // lower bound: particle index n -> global index n -> bound ID 2n + 0
    // upper bound: particle index n -> global index n -> bound ID 2n + 1
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++)
    {

        // get particle position
        double pos_y_i = sphere_pvs.position_y_vec[indx_i];

        // get particle radius
        int type_i = sphere_pvs.type_vec[indx_i];
        double rad_i = radius_vec[type_i];

        // calculate lower and uppper bounds
        bound_y_vec.push_back(pos_y_i - (1.+enlarge_factor)*rad_i);
        bound_y_vec.push_back(pos_y_i + (1.+enlarge_factor)*rad_i);

    }

    // calculate lower and upper bounds of mesh
    // lower bound: mesh index n -> global index num_particle + n -> bound ID 2*num_particle + 2n + 0
    // upper bound: mesh index n -> global index num_particle + n -> bound ID 2*num_particle + 2n + 1
    for (int indx_k = 0; indx_k < wallmesh_pvs.num_mesh; indx_k++)
    {

        // get mesh points
        double pos_p1_y_k = wallmesh_pvs.position_p1_y_vec[indx_k];
        double pos_p2_y_k = wallmesh_pvs.position_p2_y_vec[indx_k];
        double pos_p3_y_k = wallmesh_pvs.position_p3_y_vec[indx_k];

        // get min and max points
        double pos_y_min_k = pos_p1_y_k;
        if (pos_y_min_k > pos_p2_y_k) {pos_y_min_k = pos_p2_y_k;}
        if (pos_y_min_k > pos_p3_y_k) {pos_y_min_k = pos_p3_y_k;}
        double pos_y_max_k = pos_p1_y_k;
        if (pos_y_max_k < pos_p2_y_k) {pos_y_max_k = pos_p2_y_k;}
        if (pos_y_max_k < pos_p3_y_k) {pos_y_max_k = pos_p3_y_k;}

        // get width
        double pos_y_width_k = pos_y_max_k - pos_y_min_k;

        // calculate lower and uppper bounds
        bound_y_vec.push_back(pos_y_min_k - enlarge_factor*pos_y_width_k);
        bound_y_vec.push_back(pos_y_max_k + enlarge_factor*pos_y_width_k);

    }

    // initialize vector of pairs
    // first -> bound index
    // second -> bound coordinate
    int num_bound_b = 2*sphere_pvs.num_particle + 2*wallmesh_pvs.num_mesh;
    VectorPairIntDouble bound_id_pos_y_vec;

    // reset vector if particle count changed
    if (sphere_pvs.num_particle != num_particle_old || sphere_pvs.num_particle_historical_max != num_particle_historical_max_old)
    {

        // redo particle IDs
        bound_rank_id_b_vec.clear();
        for (int rank = 0; rank < num_bound_b; rank++)
        {
            bound_rank_id_b_vec.push_back(rank);
        }

        // record number of bounds
        num_particle_old = sphere_pvs.num_particle;
        num_particle_historical_max_old = sphere_pvs.num_particle_historical_max;

    }

    // generate vector of pairs
    for (int rank = 0; rank < num_bound_b; rank++)
    {

        // use arangement of ids from previous iteration
        // speeds up insertion sort
        int id_b = bound_rank_id_b_vec[rank];

        // generate pairs
        std::pair<int, double> id_pos_y_sub = {id_b, bound_y_vec[id_b]};
        bound_id_pos_y_vec.push_back(id_pos_y_sub);

    }

    // sort according to values
    sort_pair_insertion(bound_id_pos_y_vec);

    // initialize bound id to rank vectors
    // input (index): bound id; output (value): bound rank
    VectorInt bound_id_rank_b_vec(num_bound_b, 0);

    // generate vector of pairs
    for (int rank = 0; rank < num_bound_b; rank++)
    {

        // get id corresponding to each rank
        int id_b = bound_id_pos_y_vec[rank].first;

        // fill bound rank to id vector
        bound_rank_id_b_vec[rank] = id_b;

        // fill bound id to rank vector
        bound_id_rank_b_vec[id_b] = rank;

    }

    // initialize for sweep
    VectorPairInt indx_checkpair_y_vec;  // vector with preliminary collision pairs
    VectorInt indx_active_y_vec;  // vector with particle indices in "active zone"

    // sweep from lower to higher x
    for (int rank = 0; rank < num_bound_b; rank++)
    {

        // get bound ID
        int id_b = bound_rank_id_b_vec[rank];

        // calculate global index, object type, and bound type
        int indx_i = id_b / 2;
        bool is_lower_bound = (id_b % 2 == 0);

        // if lower bound -> add to active zone; create new collision pairs
        // if upper bound -> remove from active zone
        if (is_lower_bound)
        {
            
            // add to active zone
            indx_active_y_vec.push_back(indx_i);

            // skip if no pairs
            int num_active = indx_active_y_vec.size();
            if (num_active < 2)
            {
                continue;
            }

            // create collision pairs
            for (int i = 0; i < num_active - 1; i++)
            {
                
                // collision is between indx_active_y_vec[i] and indx_i
                // skip if both particles or both mesh
                bool is_both_particle = indx_active_y_vec[i] < sphere_pvs.num_particle && indx_i < sphere_pvs.num_particle;
                bool is_both_mesh = indx_active_y_vec[i] >= sphere_pvs.num_particle && indx_i >= sphere_pvs.num_particle;
                if (is_both_particle || is_both_mesh)
                {
                    continue;
                }

                // put smaller global index (particle) first
                // revert to particle and mesh indices
                std::pair<int, int> pair_sub = {indx_active_y_vec[i], indx_i - sphere_pvs.num_particle};
                if (indx_active_y_vec[i] > indx_i)
                {
                    pair_sub = {indx_i, indx_active_y_vec[i] - sphere_pvs.num_particle};
                }
                indx_checkpair_y_vec.push_back(pair_sub);

            }

        }
        else
        {
            
            // remove particle from active zone
            auto it = std::find(indx_active_y_vec.begin(), indx_active_y_vec.end(), indx_i);
            indx_active_y_vec.erase(it); 
        
        }

    }

    // sweep along x-axis

    // initialize for sweep
    VectorPairInt indx_checkpair_xy_vec;  // vector with preliminary collision pairs

    // iterate through each pair from previous preliminary list
    for (auto &pair_sub : indx_checkpair_y_vec)
    {

        // get particle and mesh indices
        int indx_i = pair_sub.first;
        int indx_k = pair_sub.second;

        // get particle position
        double pos_x_i = sphere_pvs.position_x_vec[indx_i];

        // get particle radius
        int type_i = sphere_pvs.type_vec[indx_i];
        double rad_i = radius_vec[type_i];

        // calculate lower and uppper bounds
        double lower_bound_x_i = pos_x_i - (1.+enlarge_factor)*rad_i;
        double upper_bound_x_i = pos_x_i + (1.+enlarge_factor)*rad_i;

        // get mesh points
        double pos_p1_x_k = wallmesh_pvs.position_p1_x_vec[indx_k];
        double pos_p2_x_k = wallmesh_pvs.position_p2_x_vec[indx_k];
        double pos_p3_x_k = wallmesh_pvs.position_p3_x_vec[indx_k];

        // get min and max points
        double pos_x_min_k = pos_p1_x_k;
        if (pos_x_min_k > pos_p2_x_k) {pos_x_min_k = pos_p2_x_k;}
        if (pos_x_min_k > pos_p3_x_k) {pos_x_min_k = pos_p3_x_k;}
        double pos_x_max_k = pos_p1_x_k;
        if (pos_x_max_k < pos_p2_x_k) {pos_x_max_k = pos_p2_x_k;}
        if (pos_x_max_k < pos_p3_x_k) {pos_x_max_k = pos_p3_x_k;}

        // calculate lower and uppper bounds
        double pos_x_width_k = pos_x_max_k - pos_x_min_k;
        double lower_bound_x_k = pos_x_min_k - enlarge_factor*pos_x_width_k;
        double upper_bound_x_k = pos_x_max_k + enlarge_factor*pos_x_width_k;

        // check if overlap
        bool is_overlap_x = (
            (lower_bound_x_i <= lower_bound_x_k && lower_bound_x_k <= upper_bound_x_i) ||
            (lower_bound_x_i <= upper_bound_x_k && upper_bound_x_k <= upper_bound_x_i) ||
            (lower_bound_x_k <= lower_bound_x_i && lower_bound_x_i <= upper_bound_x_k) ||
            (lower_bound_x_k <= upper_bound_x_i && upper_bound_x_i <= upper_bound_x_k)
        );

        // store pair if overlap occurs
        if (is_overlap_x)
        {
            indx_checkpair_xy_vec.push_back(pair_sub);
        }
        
    }

    // sweep along z-axis

    // initialize for sweep
    VectorPairInt indx_checkpair_xyz_vec;  // vector with preliminary collision pairs

    // iterate through each pair from previous preliminary list
    for (auto &pair_sub : indx_checkpair_xy_vec)
    {

        // get particle and mesh indices
        int indx_i = pair_sub.first;
        int indx_k = pair_sub.second;

        // get particle position
        double pos_z_i = sphere_pvs.position_z_vec[indx_i];

        // get particle radius
        int type_i = sphere_pvs.type_vec[indx_i];
        double rad_i = radius_vec[type_i];

        // calculate lower and uppper bounds
        double lower_bound_z_i = pos_z_i - (1.+enlarge_factor)*rad_i;
        double upper_bound_z_i = pos_z_i + (1.+enlarge_factor)*rad_i;

        // get mesh points
        double pos_p1_z_k = wallmesh_pvs.position_p1_z_vec[indx_k];
        double pos_p2_z_k = wallmesh_pvs.position_p2_z_vec[indx_k];
        double pos_p3_z_k = wallmesh_pvs.position_p3_z_vec[indx_k];

        // get min and max points
        double pos_z_min_k = pos_p1_z_k;
        if (pos_z_min_k > pos_p2_z_k) {pos_z_min_k = pos_p2_z_k;}
        if (pos_z_min_k > pos_p3_z_k) {pos_z_min_k = pos_p3_z_k;}
        double pos_z_max_k = pos_p1_z_k;
        if (pos_z_max_k < pos_p2_z_k) {pos_z_max_k = pos_p2_z_k;}
        if (pos_z_max_k < pos_p3_z_k) {pos_z_max_k = pos_p3_z_k;}

        // calculate lower and uppper bounds
        double pos_z_width_k = pos_z_max_k - pos_z_min_k;
        double lower_bound_z_k = pos_z_min_k - enlarge_factor*pos_z_width_k;
        double upper_bound_z_k = pos_z_max_k + enlarge_factor*pos_z_width_k;

        // check if overlap
        bool is_overlap_z = (
            (lower_bound_z_i <= lower_bound_z_k && lower_bound_z_k <= upper_bound_z_i) ||
            (lower_bound_z_i <= upper_bound_z_k && upper_bound_z_k <= upper_bound_z_i) ||
            (lower_bound_z_k <= lower_bound_z_i && lower_bound_z_i <= upper_bound_z_k) ||
            (lower_bound_z_k <= upper_bound_z_i && upper_bound_z_i <= upper_bound_z_k)
        );

        // store pair if overlap occurs
        if (is_overlap_z)
        {
            indx_checkpair_xyz_vec.push_back(pair_sub);
        }
        
    }

    return indx_checkpair_xyz_vec;

}


void CollisionCheckSphereWallMeshSweep1Dy::sort_pair_insertion(VectorPairIntDouble &pair_vec)
{

    // iterate through each element
    for (int i = 1; i < pair_vec.size(); i++)
    {
        
        // select element
        std::pair<int, double> pair_sub = pair_vec[i];

        // iterate through each element behind pair_sub
        // shift elements to the right if greater than pair_sub
        // otherwise stop the loop and assign pair_sub
        int j = i - 1;
        while (j >= 0 && pair_vec[j].second > pair_sub.second)
        {
            pair_vec[j + 1] = pair_vec[j];
            j = j - 1;
        }

        // assign pair_sub
        pair_vec[j + 1] = pair_sub;

    }

}

#endif
