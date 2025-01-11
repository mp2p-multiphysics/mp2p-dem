#ifndef PHYSICS_SPHERE_INSERT_AT_CSV
#define PHYSICS_SPHERE_INSERT_AT_CSV
#include <fstream>
#include <sstream>
#include <vector>
#include "physics_base.hpp"
#include "particle_sphere.hpp"

namespace DEM
{

class PhysicsSphereInsertAtCSV : public PhysicsBase
{

    public:

    // particle object to insert to
    ParticleSphere* sphere_ptr;
    double scale_factor = 0.;

    // insertion times
    int ts_insert = 0;

    // data aobut particles to be inserted
    int num_element = 0;
    VectorInt material_vec;
    std::vector<EigenVector3D> position_vec;
    std::vector<EigenVector3D> velocity_vec;
    std::vector<EigenVector3D> angularposition_vec;
    std::vector<EigenVector3D> angularvelocity_vec;
    VectorDouble radius_vec;

    // file names
    std::string file_in_str;

    // functions used in timestepping
    void compute_force(int ts) {};
    void compute_position_velocity(int ts) {};
    void compute_insert_delete(int ts);

    // default constructor
    PhysicsSphereInsertAtCSV() {}

    // constructor
    PhysicsSphereInsertAtCSV(ParticleSphere &sphere_in, int ts_insert_in, std::string file_in_str_in, double scale_factor_in = 1.0)
    {
        
        // store particle objects
        sphere_ptr = &sphere_in;
        scale_factor = scale_factor_in;

        // store insertion times
        ts_insert = ts_insert_in;

        // store file names
        file_in_str = file_in_str_in;

        // read csv files
        read_csv();

    }

    private:
    
    // functions
    void read_csv();

};

void PhysicsSphereInsertAtCSV::compute_insert_delete(int ts)
{

    // skip if no insertion
    if (ts != ts_insert)
    {
        return;
    }

    // record where to start new tid and gid
    int tid_last = sphere_ptr->num_element;
    int pid_last = sphere_ptr->num_element_max;

    // append particle data
    sphere_ptr->num_element += num_element;
    sphere_ptr->num_element_max += num_element;
    sphere_ptr->material_vec.insert(sphere_ptr->material_vec.end(), material_vec.begin(), material_vec.end());
    sphere_ptr->position_vec.insert(sphere_ptr->position_vec.end(), position_vec.begin(), position_vec.end());
    sphere_ptr->velocity_vec.insert(sphere_ptr->velocity_vec.end(), velocity_vec.begin(), velocity_vec.end());
    sphere_ptr->angularposition_vec.insert(sphere_ptr->angularposition_vec.end(), angularposition_vec.begin(), angularposition_vec.end());
    sphere_ptr->angularvelocity_vec.insert(sphere_ptr->angularvelocity_vec.end(), angularvelocity_vec.begin(), angularvelocity_vec.end());
    sphere_ptr->radius_vec.insert(sphere_ptr->radius_vec.end(), radius_vec.begin(), radius_vec.end());

    // reindex pid and gid
    for (int indx_i = 0; indx_i < num_element; indx_i++)
    {
        sphere_ptr->tid_to_pid_vec.push_back(pid_last + indx_i);
        sphere_ptr->pid_to_tid_map[pid_last + indx_i] = tid_last;
    }

}

void PhysicsSphereInsertAtCSV::read_csv()
{

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

    // initialize for iteration
    bool is_header = true;  // true while reading header
    std::string line_str;  // stores lines in files

    // iterate for each line in the file
    while (std::getline(file_in_stream, line_str))
    {

        // skip header
        if (is_header)
        {
            is_header = false; // not reading header
            continue;
        }

        // count number of particles
        num_element++;

        // convert line string into stringstream
        std::stringstream line_stream(line_str);
    
        // initialize for iteration
        int value_num = 0;  // counts position of value
        std::string value_str;  // stores values in lines

        // iterate through each value
        while (std::getline(line_stream, value_str, ','))
        {

            // initialize coordinate vectors
            EigenVector3D position_sub;
            EigenVector3D velocity_sub;
            EigenVector3D angularposition_sub;
            EigenVector3D angularvelocity_sub;

            // store values in appropriate vector
            switch (value_num)
            {
                case 0: break; // particle ID
                case 1: material_vec.push_back(std::stoi(value_str)); break;
                case 2: position_sub.coeffRef(0) = scale_factor * std::stod(value_str); break;
                case 3: position_sub.coeffRef(1) = scale_factor * std::stod(value_str); break;
                case 4: position_sub.coeffRef(2) = scale_factor * std::stod(value_str); break;
                case 5: velocity_sub.coeffRef(0) = scale_factor * std::stod(value_str); break;
                case 6: velocity_sub.coeffRef(1) = scale_factor * std::stod(value_str); break;
                case 7: velocity_sub.coeffRef(2) = scale_factor * std::stod(value_str); break;
                case 8: angularposition_sub.coeffRef(0) = std::stod(value_str); break;
                case 9: angularposition_sub.coeffRef(1) = std::stod(value_str); break;
                case 10: angularposition_sub.coeffRef(2) = std::stod(value_str); break;
                case 11: angularvelocity_sub.coeffRef(0) = std::stod(value_str); break;
                case 12: angularvelocity_sub.coeffRef(1) = std::stod(value_str); break;
                case 13: angularvelocity_sub.coeffRef(2) = std::stod(value_str); break;
                case 14: radius_vec.push_back(scale_factor * std::stod(value_str)); break;
            }

            // insert coordinate vectors
            position_vec.push_back(position_sub);
            velocity_vec.push_back(velocity_sub);
            angularposition_vec.push_back(angularposition_sub);
            angularvelocity_vec.push_back(angularvelocity_sub);

            // increment value count
            value_num++;

        }
    
    }

    // close file
    file_in_stream.close();

}

}

#endif
