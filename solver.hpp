#ifndef SOLVER
#define SOLVER
#include <iostream>
#include <unordered_set>
#include "container_typedef.hpp"
#include "forcemoment_base.hpp"
#include "insertdelete_base.hpp"
#include "positionvelocity_base.hpp"

namespace DEM
{

class Solver
{

    public:

    // vectors
    std::vector<InsertDeleteBase*> insertdelete_ptr_vec;
    std::vector<ForceMomentBase*> forcemoment_ptr_vec;
    std::vector<PositionVelocityBase*> positionvelocity_ptr_vec;
    std::vector<BaseGroup*> group_ptr_vec;

    // timestepping
    int num_ts = 0;
    int num_ts_output = 0;
    double dt = 0.;

    // functions
    void set_timestep(int num_ts_in, int num_ts_output_in, double dt_in);
    void solve(bool verbose);

    // default constructor
    Solver() {}    

    // constructor
    Solver(std::vector<InsertDeleteBase*> insertdelete_ptr_vec_in, std::vector<ForceMomentBase*> forcemoment_ptr_vec_in, std::vector<PositionVelocityBase*> positionvelocity_ptr_vec_in)
    {

        // store vectors
        insertdelete_ptr_vec = insertdelete_ptr_vec_in;
        forcemoment_ptr_vec = forcemoment_ptr_vec_in;
        positionvelocity_ptr_vec = positionvelocity_ptr_vec_in;

        // get groups
        extract_group_vec();

    }

    private:

    // functions
    void extract_group_vec();

};

void Solver::set_timestep(int num_ts_in, int num_ts_output_in, double dt_in)
{

    // set timestepping parameters
    num_ts = num_ts_in;
    num_ts_output = num_ts_output_in;
    dt = dt_in;

}

void Solver::solve(bool verbose = true)
{

    // iterate through each timestep
    for (int ts = 0; ts < num_ts; ts++)
    {

        // clear forces and moments
        for (auto group_ptr : group_ptr_vec)
        {
            group_ptr->clear_forcemoment();
        }

        // insertdelete -> forcemoment -> positionvelocity
        for (auto insertdelete_ptr : insertdelete_ptr_vec)
        {
            insertdelete_ptr->update(ts, dt);
        }

        // write output files
        if (ts % num_ts_output == 0){
        for (auto group_ptr : group_ptr_vec){
            group_ptr->write_output(ts);
        }}

        for (auto forcemoment_ptr : forcemoment_ptr_vec)
        {
            forcemoment_ptr->update(ts, dt);
        }
        for (auto positionvelocity_ptr : positionvelocity_ptr_vec)
        {
            positionvelocity_ptr->update(ts, dt);
        }

        // output progress
        if (verbose && ts % num_ts_output == 0)
        {
            std::cout << "Timestep: " << ts << "\n";
        }

    }

}

void Solver::extract_group_vec()
{

    // initialize set of groups
    std::unordered_set<BaseGroup*> group_ptr_set;

    // iterate through objects
    for (auto insertdelete_ptr : insertdelete_ptr_vec)
    {
        std::vector<BaseGroup*> group_vec = insertdelete_ptr->get_group_ptr_vec();
        group_ptr_set.insert(group_vec.begin(), group_vec.end());
    }
    for (auto forcemoment_ptr : forcemoment_ptr_vec)
    {
        std::vector<BaseGroup*> group_vec = forcemoment_ptr->get_group_ptr_vec();
        group_ptr_set.insert(group_vec.begin(), group_vec.end());
    }
    for (auto positionvelocity_ptr : positionvelocity_ptr_vec)
    {
        std::vector<BaseGroup*> group_vec = positionvelocity_ptr->get_group_ptr_vec();
        group_ptr_set.insert(group_vec.begin(), group_vec.end());
    }

    // convert to vector
    group_ptr_vec = std::vector<BaseGroup*>(group_ptr_set.begin(), group_ptr_set.end());

}

}

#endif
