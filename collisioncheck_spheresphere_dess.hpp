#ifndef COLLISIONCHECK_SPHERESPHERE_DESS
#define COLLISIONCHECK_SPHERESPHERE_DESS
#include <set>
#include <utility>
#include <vector>
#include "container_sphere.hpp"
#include "container_typedef.hpp"

class CollisionCheckSphereSphereDESS
{

    public:

    // variables
    VectorDouble radius_vec;

    // functions
    void set_input_parameter(VectorDouble radius_vec_in);
    VectorPairInt broad_search(SpherePositionVelocityStruct &sphere_pvs);

    // default constructor
    CollisionCheckSphereSphereDESS()
    {

    }

    // constructor
    CollisionCheckSphereSphereDESS(VectorDouble radius_vec_in)
    {
        radius_vec = radius_vec_in;
    }

    private:

    // bound rank to id vectors
    // input (index): bound rank; output (value): bound id
    VectorInt bound_rank_id_a_vec;
    VectorInt bound_rank_id_b_vec;
    VectorInt bound_rank_id_c_vec;

    // number of particles in previous usage
    int num_particle_old = 0;
    int num_particle_historical_max_old = 0;

    // functions
    void sort_by_pair_second(VectorPairIntDouble &pair_vec);

};

void CollisionCheckSphereSphereDESS::set_input_parameter(VectorDouble radius_vec_in)
{
    radius_vec = radius_vec_in;
}

VectorPairInt CollisionCheckSphereSphereDESS::broad_search(SpherePositionVelocityStruct &sphere_pvs)
{

    // initialize vector with index pairs
    VectorPairInt collision_vec;

    // initialize vectors with lower and upper bounds
    VectorDouble bound_x_vec;
    VectorDouble bound_y_vec;
    VectorDouble bound_z_vec;

    // calculate lower and upper bounds
    // lower bound: particle index n -> bound ID 2n + 0
    // upper bound: particle index n -> bound ID 2n + 1
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++)
    {

        // get particle position
        double pos_x_i = sphere_pvs.position_x_vec[indx_i];
        double pos_y_i = sphere_pvs.position_y_vec[indx_i];
        double pos_z_i = sphere_pvs.position_z_vec[indx_i];

        // get particle radius
        int type_i = sphere_pvs.type_vec[indx_i];
        double rad_i = radius_vec[type_i];

        // calculate lower bound
        bound_x_vec.push_back(pos_x_i - rad_i);
        bound_y_vec.push_back(pos_y_i - rad_i);
        bound_z_vec.push_back(pos_z_i - rad_i);

        // calculate upper bound
        bound_x_vec.push_back(pos_x_i + rad_i);
        bound_y_vec.push_back(pos_y_i + rad_i);
        bound_z_vec.push_back(pos_z_i + rad_i);

    }

    // initialize vector of pairs
    // first -> bound index
    // second -> bound coordinate
    int num_bound = 2*sphere_pvs.num_particle;
    VectorPairIntDouble bound_id_pos_x_vec;
    VectorPairIntDouble bound_id_pos_y_vec;
    VectorPairIntDouble bound_id_pos_z_vec;

    // reset vectors if particle count changed
    if (sphere_pvs.num_particle != num_particle_old || sphere_pvs.num_particle_historical_max != num_particle_historical_max_old)
    {

        // clear vectors
        bound_rank_id_a_vec.clear();
        bound_rank_id_b_vec.clear();
        bound_rank_id_c_vec.clear();

        // more particles -> add more ids
        for (int rank = 0; rank < num_bound; rank++)
        {
            bound_rank_id_a_vec.push_back(rank);
            bound_rank_id_b_vec.push_back(rank);
            bound_rank_id_c_vec.push_back(rank);
        }

        // record number of bounds
        num_particle_old = sphere_pvs.num_particle;
        num_particle_historical_max_old = sphere_pvs.num_particle_historical_max;

    }

    // generate vector of pairs
    for (int rank = 0; rank < num_bound; rank++)
    {

        // use arangement of ids from previous iteration
        // speeds up insertion sort
        int id_a = bound_rank_id_a_vec[rank];
        int id_b = bound_rank_id_b_vec[rank];
        int id_c = bound_rank_id_c_vec[rank];

        // generate pairs
        std::pair<int, double> id_pos_x_sub = {id_a, bound_x_vec[id_a]};
        std::pair<int, double> id_pos_y_sub = {id_b, bound_y_vec[id_b]};
        std::pair<int, double> id_pos_z_sub = {id_c, bound_z_vec[id_c]};

        // append to vector
        bound_id_pos_x_vec.push_back(id_pos_x_sub);
        bound_id_pos_y_vec.push_back(id_pos_y_sub);
        bound_id_pos_z_vec.push_back(id_pos_z_sub);

    }

    // sort according to values
    sort_by_pair_second(bound_id_pos_x_vec);
    sort_by_pair_second(bound_id_pos_y_vec);
    sort_by_pair_second(bound_id_pos_z_vec);

    // initialize bound rank to id vectors
    // input (index): bound rank; output (value): bound id
    // VectorInt bound_rank_id_a_vec(num_bound, 0);  // x -> a
    // VectorInt bound_rank_id_b_vec(num_bound, 0);  // y -> b
    // VectorInt bound_rank_id_c_vec(num_bound, 0);  // z -> c

    // initialize bound id to rank vectors
    // input (index): bound id; output (value): bound rank
    VectorInt bound_id_rank_a_vec(num_bound, 0);
    VectorInt bound_id_rank_b_vec(num_bound, 0);
    VectorInt bound_id_rank_c_vec(num_bound, 0);

    // generate vector of pairs
    for (int rank = 0; rank < num_bound; rank++)
    {

        // get id corresponding to each rank
        int id_a = bound_id_pos_x_vec[rank].first;
        int id_b = bound_id_pos_y_vec[rank].first;
        int id_c = bound_id_pos_z_vec[rank].first;

        // fill bound rank to id vectors
        bound_rank_id_a_vec[rank] = id_a;
        bound_rank_id_b_vec[rank] = id_b;
        bound_rank_id_c_vec[rank] = id_c;

        // fill bound id to rank vectors
        bound_id_rank_a_vec[id_a] = rank;
        bound_id_rank_b_vec[id_b] = rank;
        bound_id_rank_c_vec[id_c] = rank;

    }

    // iterate through each particle i
    for (int indx_i = 0; indx_i < sphere_pvs.num_particle; indx_i++)
    {

        // get ids of lower and upper bounds along x-axis
        int id_a_lower = 2*indx_i;
        int id_a_upper = 2*indx_i + 1;

        // get ranks of lower and upper bounds along x-axis
        int rank_a_lower = bound_id_rank_a_vec[id_a_lower];
        int rank_a_upper = bound_id_rank_a_vec[id_a_upper];

        // initialize set of particle j (possible collision)
        // checked against x axis
        std::set<int> indx_j_checkx_set;

        // iterate through bound ids between rank_a_lower and upper
        // record corresponding particle indices
        for (int rank_a = rank_a_lower+1; rank_a < rank_a_upper; rank_a++)
        {
            int id_a = bound_rank_id_a_vec[rank_a];  // bound id
            indx_j_checkx_set.insert(id_a/2);  // convert to particle index
        }

        // initialize set of particle j (possible collision)
        // checked against x and y axes
        std::set<int> indx_j_checkxy_set;

        // iterate through y-axis
        // for each particle j, check if its bounds surrounds particle i's bounds
        for (auto &indx_j : indx_j_checkx_set)
        {

            // get ids of lower and upper bounds along y-axis
            int id_b_lower = 2*indx_j;
            int id_b_upper = 2*indx_j + 1;

            // get ranks of lower and upper bounds along y-axis
            int rank_b_lower = bound_id_rank_b_vec[id_b_lower];
            int rank_b_upper = bound_id_rank_b_vec[id_b_upper];

            // iterate through bound ids between rank_b_lower and upper
            // if particle i's bounds are found, then collision is possible
            for (int rank_b = rank_b_lower+1; rank_b < rank_b_upper; rank_b++)
            {
                int id_b = bound_rank_id_b_vec[rank_b];  // bound id
                if (id_b/2 == indx_i)  // convert to particle id
                {
                    indx_j_checkxy_set.insert(indx_j);
                }
            }

        }

        // initialize set of particle j (possible collision)
        // checked against x, y, and z axes
        std::set<int> indx_j_checkxyz_set;

        // iterate through z-axis
        // for each particle j, check if its bounds surrounds particle i's bounds
        for (auto &indx_j : indx_j_checkxy_set)
        {

            // get ids of lower and upper bounds along y-axis
            int id_c_lower = 2*indx_j;
            int id_c_upper = 2*indx_j + 1;

            // get ranks of lower and upper bounds along y-axis
            int rank_c_lower = bound_id_rank_c_vec[id_c_lower];
            int rank_c_upper = bound_id_rank_c_vec[id_c_upper];

            // iterate through bound ids between rank_c_lower and upper
            // if particle i's bounds are found, then collision is possible
            for (int rank_c = rank_c_lower+1; rank_c < rank_c_upper; rank_c++)
            {
                int id_c = bound_rank_id_c_vec[rank_c];  // bound id
                if (id_c/2 == indx_i)  // convert to particle id
                {
                    indx_j_checkxyz_set.insert(indx_j);
                }
            }

        }

        // generate pairs for collision checking
        for (auto &indx_j : indx_j_checkxyz_set)
        {
            
            // create collision pair
            std::pair<int, int> pair_sub = {indx_i, indx_j};

            // append to vector of collision pairs
            collision_vec.push_back(pair_sub);

        }

    }

    return collision_vec;

}

void CollisionCheckSphereSphereDESS::sort_by_pair_second(VectorPairIntDouble &pair_vec)
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
