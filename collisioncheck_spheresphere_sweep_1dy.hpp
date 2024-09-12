#ifndef COLLISIONCHECK_SPHERESPHERE_SWEEP_1DY
#define COLLISIONCHECK_SPHERESPHERE_SWEEP_1DY
#include <set>
#include <utility>
#include <vector>
#include "container_sphere.hpp"
#include "container_typedef.hpp"

class CollisionCheckSphereSphereSweep1Dy
{

    public:

    // variables
    VectorDouble radius_vec;
    double enlarge_factor;

    // functions
    void set_input(VectorDouble &radius_vec_in);
    VectorPairInt broad_search(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    CollisionCheckSphereSphereSweep1Dy()
    {

    }

    // constructor
    CollisionCheckSphereSphereSweep1Dy(VectorDouble radius_vec_in, double enlarge_factor_in = 0.05)
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

void CollisionCheckSphereSphereSweep1Dy::set_input(VectorDouble &radius_vec_in)
{

    // set input variables
    radius_vec = radius_vec_in;

}

VectorPairInt CollisionCheckSphereSphereSweep1Dy::broad_search(SpherePositionVelocityStruct &sphere_pvs)
{

    // sweep along y axis

    // initialize vector with lower and upper bound
    VectorDouble bound_y_vec;

    // calculate lower and upper bounds
    // lower bound: particle index n -> bound ID 2n + 0
    // upper bound: particle index n -> bound ID 2n + 1
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

    // initialize vector of pairs
    // first -> bound index
    // second -> bound coordinate
    int num_bound_b = 2*sphere_pvs.num_particle;
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
        int id_a = bound_id_pos_y_vec[rank].first;

        // fill bound rank to id vector
        bound_rank_id_b_vec[rank] = id_a;

        // fill bound id to rank vector
        bound_id_rank_b_vec[id_a] = rank;

    }

    // initialize for sweep
    VectorPairInt indx_checkpair_y_vec;  // vector with preliminary collision pairs
    VectorInt indx_active_y_vec;  // vector with particle indices in "active zone"

    // sweep from lower to higher x
    for (int rank = 0; rank < num_bound_b; rank++)
    {

        // get bound ID
        int id_b = bound_rank_id_b_vec[rank];

        // calculate particle index and bound type
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
                std::pair<int, int> pair_sub = {indx_active_y_vec[i], indx_i};
                if (indx_active_y_vec[i] > indx_i)
                {
                    pair_sub = {indx_i, indx_active_y_vec[i]};
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

        // get particle indices
        int indx_i = pair_sub.first;
        int indx_j = pair_sub.second;

        // get particle position
        double pos_x_i = sphere_pvs.position_x_vec[indx_i];
        double pos_x_j = sphere_pvs.position_x_vec[indx_j];

        // get particle radius
        int type_i = sphere_pvs.type_vec[indx_i];
        int type_j = sphere_pvs.type_vec[indx_j];
        double rad_i = radius_vec[type_i];
        double rad_j = radius_vec[type_j];

        // calculate lower and uppper bounds
        double lower_bound_x_i = pos_x_i - (1.+enlarge_factor)*rad_i;
        double lower_bound_x_j = pos_x_j - (1.+enlarge_factor)*rad_j;
        double upper_bound_x_i = pos_x_i + (1.+enlarge_factor)*rad_i;
        double upper_bound_x_j = pos_x_j + (1.+enlarge_factor)*rad_j;

        // check if overlap
        bool is_overlap_x = (
            (lower_bound_x_i <= lower_bound_x_j && lower_bound_x_j <= upper_bound_x_i) ||
            (lower_bound_x_i <= upper_bound_x_j && upper_bound_x_j <= upper_bound_x_i) ||
            (lower_bound_x_j <= lower_bound_x_i && lower_bound_x_i <= upper_bound_x_j) ||
            (lower_bound_x_j <= upper_bound_x_i && upper_bound_x_i <= upper_bound_x_j)
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

        // get particle indices
        int indx_i = pair_sub.first;
        int indx_j = pair_sub.second;

        // get particle position
        double pos_z_i = sphere_pvs.position_z_vec[indx_i];
        double pos_z_j = sphere_pvs.position_z_vec[indx_j];

        // get particle radius
        int type_i = sphere_pvs.type_vec[indx_i];
        int type_j = sphere_pvs.type_vec[indx_j];
        double rad_i = radius_vec[type_i];
        double rad_j = radius_vec[type_j];

        // calculate lower and uppper bounds
        double lower_bound_z_i = pos_z_i - (1.+enlarge_factor)*rad_i;
        double lower_bound_z_j = pos_z_j - (1.+enlarge_factor)*rad_j;
        double upper_bound_z_i = pos_z_i + (1.+enlarge_factor)*rad_i;
        double upper_bound_z_j = pos_z_j + (1.+enlarge_factor)*rad_j;

        // check if overlap
        bool is_overlap_z = (
            (lower_bound_z_i <= lower_bound_z_j && lower_bound_z_j <= upper_bound_z_i) ||
            (lower_bound_z_i <= upper_bound_z_j && upper_bound_z_j <= upper_bound_z_i) ||
            (lower_bound_z_j <= lower_bound_z_i && lower_bound_z_i <= upper_bound_z_j) ||
            (lower_bound_z_j <= upper_bound_z_i && upper_bound_z_i <= upper_bound_z_j)
        );

        // store pair if overlap occurs
        if (is_overlap_z)
        {
            indx_checkpair_xyz_vec.push_back(pair_sub);
        }
        
    }

    return indx_checkpair_xyz_vec;

}

void CollisionCheckSphereSphereSweep1Dy::sort_pair_insertion(VectorPairIntDouble &pair_vec)
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
