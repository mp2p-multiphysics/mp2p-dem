#ifndef SOLVER
#define SOLVER
#include <iostream>
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

    // timestepping
    int num_ts = 0;
    double dt = 0.;

    // functions
    void set_timestep(int num_ts_in, double dt_in);
    void solve(int num_ts_output, bool verbose);

    // default constructor
    Solver() {}    

    // constructor
    Solver(std::vector<InsertDeleteBase*> insertdelete_ptr_vec_in, std::vector<ForceMomentBase*> forcemoment_ptr_vec_in, std::vector<PositionVelocityBase*> positionvelocity_ptr_vec_in)
    {

        // store vectors
        insertdelete_ptr_vec = insertdelete_ptr_vec_in;
        forcemoment_ptr_vec = forcemoment_ptr_vec_in;
        positionvelocity_ptr_vec = positionvelocity_ptr_vec_in;

    }

    private:

};

void Solver::set_timestep(int num_ts_in, double dt_in)
{

    // set timestepping parameters
    num_ts = num_ts_in;
    dt = dt_in;

}

void Solver::solve(int num_ts_output = 100, bool verbose = true)
{

    // iterate through each timestep
    for (int ts = 0; ts < num_ts; ts++)
    {

        // insertdelete -> forcemoment -> positionvelocity
        for (auto insertdelete_ptr : insertdelete_ptr_vec)
        {
            insertdelete_ptr->update(ts, dt);
        }
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

}

#endif
