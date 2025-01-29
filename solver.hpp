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
    /*

    Runs the DEM simulation.

    Functions
    =========
    set_insertdelete : void
        Set insertion or deletion objects in the simulation.
    set_modify : void
        Set modifying objects in the simulation.
    set_forcemoment : void
        Set force or moment objects in the simulation.
    set_integral : void
        Set integral objects in the simulation.
    set_timestep : void
        Set parameters needed for timestepping when running the simulation.
    solve : void
        Runs the simulation.

    */

    public:

    // vectors of calculation conditions
    std::vector<InsertDeleteBase*> insertdelete_ptr_vec;
    std::vector<ModifyBase*> modify_ptr_vec;
    std::vector<ForceMomentBase*> forcemoment_ptr_vec;
    std::vector<IntegralBase*> integral_ptr_vec;

    // set of groups
    std::vector<BaseGroup*> group_ptr_vec;

    // timestepping
    int num_timestep = 0;
    int num_timestep_output = 0;
    double dt = 0.;

    // functions
    void set_insertdelete(std::vector<InsertDeleteBase*> insertdelete_ptr_vec_in);
    void set_modify(std::vector<ModifyBase*> modify_ptr_vec_in);
    void set_forcemoment(std::vector<ForceMomentBase*> forcemoment_ptr_vec_in);
    void set_integral(std::vector<IntegralBase*> integral_ptr_vec_in);
    void set_timestep(int num_timestep_in, int num_timestep_output_in, double dt_in);
    void solve(bool verbose);

    // default constructor
    Solver() {}

    // constructor
    Solver(SphereGroup &spheregroup_in, std::vector<MeshGroup*> meshgroup_ptr_vec_in)
    {
        
        // store groups
        group_ptr_vec.push_back(&spheregroup_in);
        for (auto meshgroup_ptr : meshgroup_ptr_vec_in)
        {
            group_ptr_vec.push_back(meshgroup_ptr);
        }

    }

    private:

};

void Solver::set_insertdelete(std::vector<InsertDeleteBase*> insertdelete_ptr_vec_in)
{
    /*

    Set insertion or deletion objects in the simulation.

    Arguments
    =========
    insertdelete_ptr_vec_in : vector<InsertDeleteBase*>
        Vector of pointers to insertion or deletion objects.

    Returns
    =======
    (none)

    */

    // store objects
    insertdelete_ptr_vec = insertdelete_ptr_vec_in;

}

void Solver::set_modify(std::vector<ModifyBase*> modify_ptr_vec_in)
{
    /*

    Set modifying objects in the simulation.

    Arguments
    =========
    modify_ptr_vec_in : vector<ModifyBase*>
        Vector of pointers to modifying objects.

    Returns
    =======
    (none)

    */

    // store objects
    modify_ptr_vec = modify_ptr_vec_in;

}

void Solver::set_forcemoment(std::vector<ForceMomentBase*> forcemoment_ptr_vec_in)
{
    /*

    Set force or moment objects in the simulation.

    Arguments
    =========
    forcemoment_ptr_vec_in : vector<ForceMomentBase*>
        Vector of pointers to force or moment objects.

    Returns
    =======
    (none)

    */

    // store objects
    forcemoment_ptr_vec = forcemoment_ptr_vec_in;

}

void Solver::set_integral(std::vector<IntegralBase*> integral_ptr_vec_in)
{
    /*

    Set integral objects in the simulation.

    Arguments
    =========
    integral_ptr_vec_in : vector<IntegralBase*>
        Vector of pointers to integral objects.

    Returns
    =======
    (none)

    */

    // store objects
    integral_ptr_vec = integral_ptr_vec_in;

}

void Solver::set_timestep(int num_timestep_in, int num_timestep_output_in, double dt_in)
{
    /*

    Set parameters needed for timestepping when running the simulation.

    Arguments
    =========
    num_timestep_in : int
        Number of timesteps to simulate.
    num_timestep_output_in : int
        Frequency of output file generation.
    dt_in : double
        Time interval of timestep.

    Returns
    =======
    (none)

    */

    // set timestepping parameters
    num_timestep = num_timestep_in;
    num_timestep_output = num_timestep_output_in;
    dt = dt_in;

}

void Solver::solve(bool verbose = true)
{
    /*
    
    Runs the simulation.

    Arguments
    =========
    verbose : bool
        If true, print output to console.
        Default value is true.

    */

    // start calculation timer
    auto t_begin = std::chrono::steady_clock::now();

    // calculation time
    double insertdelete_time = 0;
    double modify_time = 0;
    double forcemoment_time = 0;
    double inputoutput_time = 0;
    double integral_time = 0;

    // initialize groups
    // insert/delete -> modify -> force/moment -> output -> integral
    auto t00 = std::chrono::steady_clock::now();
    for (auto insertdelete_ptr : insertdelete_ptr_vec)
    {
        insertdelete_ptr->initialize(dt);
    }
    auto t01 = std::chrono::steady_clock::now();
    for (auto modify_ptr : modify_ptr_vec)
    {
        modify_ptr->initialize(dt);
    }
    auto t02 = std::chrono::steady_clock::now();
    for (auto forcemoment_ptr : forcemoment_ptr_vec)
    {
        forcemoment_ptr->initialize(dt);
    }
    auto t03 = std::chrono::steady_clock::now();
    for (auto integral_ptr : integral_ptr_vec)
    {
        integral_ptr->initialize(dt);
    }
    auto t04 = std::chrono::steady_clock::now();

    // compute times
    insertdelete_time += std::chrono::duration<double>(t01 - t00).count();
    modify_time += std::chrono::duration<double>(t02 - t01).count();
    forcemoment_time += std::chrono::duration<double>(t03 - t02).count();
    integral_time += std::chrono::duration<double>(t04 - t03).count();

    // iterate through each timestep
    for (int ts = 0; ts < num_timestep; ts++)
    {
        
        // insert/delete -> modify -> force/moment -> output -> integral
        auto t0 = std::chrono::steady_clock::now();
        for (auto insertdelete_ptr : insertdelete_ptr_vec)
        {
            insertdelete_ptr->update(ts);
        }
        auto t1 = std::chrono::steady_clock::now();
        for (auto modify_ptr : modify_ptr_vec)
        {
            modify_ptr->update(ts);
        }
        auto t2 = std::chrono::steady_clock::now();
        for (auto forcemoment_ptr : forcemoment_ptr_vec)
        {
            forcemoment_ptr->update(ts);
        }
        auto t3 = std::chrono::steady_clock::now();
        if (ts % num_timestep_output == 0){
        for (auto group_ptr : group_ptr_vec){
            group_ptr->output_file(ts);
        }}
        auto t4 = std::chrono::steady_clock::now();
        for (auto integral_ptr : integral_ptr_vec)
        {
            integral_ptr->update(ts);
        }
        auto t5 = std::chrono::steady_clock::now();
        if (verbose && ts % num_timestep_output == 0)
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
    if (verbose)
    {
        std::cout << "Calculation completed in " << std::chrono::duration<double>(t_end-t_begin).count() << " s.\n";
        std::cout << "Insert/Delete : " << insertdelete_time << " s\n";
        std::cout << "Modify        : " << modify_time << " s\n";
        std::cout << "Force/Moment  : " << forcemoment_time << " s\n";
        std::cout << "Input/Output  : " << inputoutput_time << " s\n";
        std::cout << "Integral      : " << integral_time << " s\n";
    }
    
}

}

#endif
