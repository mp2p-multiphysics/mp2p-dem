#ifndef SOLVER
#define SOLVER
#include <chrono>
#include <iostream>
#include <unordered_set>
#include "container_typedef.hpp"
#include "forcemoment_base.hpp"
#include "insertdelete_base.hpp"
#include "integral_base.hpp"
#include "modify_base.hpp"

namespace DEM
{

class Solver
{

    public:

    // vectors of calculation conditions
    std::vector<InsertDeleteBase*> insertdelete_ptr_vec;
    std::vector<ModifyBase*> modify_ptr_vec;
    std::vector<ForceMomentBase*> forcemoment_ptr_vec;
    std::vector<IntegralBase*> integral_ptr_vec;

    // set of groups
    std::unordered_set<BaseGroup*> group_ptr_set;

    // timestepping
    int num_ts = 0;
    int num_ts_output = 0;
    double dt = 0.;

    // functions
    void set_insertdelete(std::vector<InsertDeleteBase*> insertdelete_ptr_vec_in);
    void set_modify(std::vector<ModifyBase*> modify_ptr_vec_in);
    void set_forcemoment(std::vector<ForceMomentBase*> forcemoment_ptr_vec_in);
    void set_integral(std::vector<IntegralBase*> integral_ptr_vec_in);
    void set_timestep(int num_ts_in, int num_ts_output_in, double dt_in);
    void solve(bool verbose);

    // default constructor
    Solver() {}    

    private:

    // functions
    void extract_group_vec();

};

void Solver::set_insertdelete(std::vector<InsertDeleteBase*> insertdelete_ptr_vec_in)
{
    
    // store objects
    insertdelete_ptr_vec = insertdelete_ptr_vec_in;
    
    // get groups
    for (auto insertdelete_ptr : insertdelete_ptr_vec)
    {
        std::vector<BaseGroup*> group_vec = insertdelete_ptr->get_group_ptr_vec();
        group_ptr_set.insert(group_vec.begin(), group_vec.end());
    }

}

void Solver::set_modify(std::vector<ModifyBase*> modify_ptr_vec_in)
{

    // store objects
    modify_ptr_vec = modify_ptr_vec_in;
    
    // get groups
    for (auto modify_ptr : modify_ptr_vec)
    {
        std::vector<BaseGroup*> group_vec = modify_ptr->get_group_ptr_vec();
        group_ptr_set.insert(group_vec.begin(), group_vec.end());
    }

}

void Solver::set_forcemoment(std::vector<ForceMomentBase*> forcemoment_ptr_vec_in)
{

    // store objects
    forcemoment_ptr_vec = forcemoment_ptr_vec_in;
    
    // get groups
    for (auto forcemoment_ptr : forcemoment_ptr_vec)
    {
        std::vector<BaseGroup*> group_vec = forcemoment_ptr->get_group_ptr_vec();
        group_ptr_set.insert(group_vec.begin(), group_vec.end());
    }

}

void Solver::set_integral(std::vector<IntegralBase*> integral_ptr_vec_in)
{

    // store objects
    integral_ptr_vec = integral_ptr_vec_in;
    
    // get groups
    for (auto integral_ptr : integral_ptr_vec)
    {
        std::vector<BaseGroup*> group_vec = integral_ptr->get_group_ptr_vec();
        group_ptr_set.insert(group_vec.begin(), group_vec.end());
    }

}

void Solver::set_timestep(int num_ts_in, int num_ts_output_in, double dt_in)
{

    // set timestepping parameters
    num_ts = num_ts_in;
    num_ts_output = num_ts_output_in;
    dt = dt_in;

}

void Solver::solve(bool verbose = true)
{

    // start calculation timer
    auto t_begin = std::chrono::steady_clock::now();

    // convert group set to vector
    std::vector<BaseGroup*> group_ptr_vec(group_ptr_set.begin(), group_ptr_set.end());

    // calculation time
    double insertdelete_time = 0;
    double modify_time = 0;
    double forcemoment_time = 0;
    double inputoutput_time = 0;
    double integral_time = 0;

    // iterate through each timestep
    for (int ts = 0; ts < num_ts; ts++)
    {

        // insert or delete objects
        auto t0 = std::chrono::steady_clock::now();
        for (auto insertdelete_ptr : insertdelete_ptr_vec)
        {
            insertdelete_ptr->update(ts, dt);
        }

        // modify objects
        auto t1 = std::chrono::steady_clock::now();
        for (auto modify_ptr : modify_ptr_vec)
        {
            modify_ptr->update(ts, dt);
        }

        // calculate forces and moments
        auto t2 = std::chrono::steady_clock::now();
        for (auto forcemoment_ptr : forcemoment_ptr_vec)
        {
            forcemoment_ptr->update(ts, dt);
        }

        // write output files
        auto t3 = std::chrono::steady_clock::now();
        if (ts % num_ts_output == 0){
        for (auto group_ptr : group_ptr_vec){
            group_ptr->output_file(ts);
        }}

        // calculate position and velocity
        // also resets forces and moments
        auto t4 = std::chrono::steady_clock::now();
        for (auto integral_ptr : integral_ptr_vec)
        {
            integral_ptr->update(ts, dt);
        }

        // output progress
        auto t5 = std::chrono::steady_clock::now();
        if (verbose && ts % num_ts_output == 0)
        {
            std::cout << "Timestep: " << ts << "\n";
        }

        // compute times
        insertdelete_time += std::chrono::duration<double>(t1 - t0).count();
        modify_time += std::chrono::duration<double>(t2 - t1).count();
        forcemoment_time += std::chrono::duration<double>(t3 - t2).count();
        inputoutput_time += std::chrono::duration<double>(t4 - t3).count();
        integral_time += std::chrono::duration<double>(t5 - t4).count();

    }

    // end calculation timer
    auto t_end = std::chrono::steady_clock::now();

    // output calculation time
    std::cout << "Calculation completed in " << std::chrono::duration<double>(t_end-t_begin).count() << " s.\n";
    std::cout << "Insert/Delete : " << insertdelete_time << " s\n";
    std::cout << "Modify        : " << modify_time << " s\n";
    std::cout << "Force/Moment  : " << forcemoment_time << " s\n";
    std::cout << "Input/Output  : " << inputoutput_time << " s\n";
    std::cout << "Integral      : " << integral_time << " s\n";
    
}

}

#endif
