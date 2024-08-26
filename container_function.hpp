#ifndef CONTAINER_FUNCTION
#define CONTAINER_FUNCTION
#include <map>
#include <utility>
#include <vector>
#include "container_typedef.hpp"


double smat_get_value(SparseMatrixDouble &smat, int i, int j)
{

    // create key for map (sparse matrix)
    std::pair<int, int> key = {i, j};

    // find key in map
    auto iterator = smat.find(key);

    // output value in map
    // key in map -> return value
    if (iterator != smat.end())  
    {
        return iterator -> second;
    }

    // key not in map -> return 0
    return 0.;

}

void smat_set_value(SparseMatrixDouble &smat, int i, int j, double value)
{

    // create key for map (sparse matrix)
    std::pair<int, int> key = {i, j};

    // assign value to map
    smat[key] = value;

}

void smat_prune(SparseMatrixDouble &smat, int i, int j)
{

    // create key for map (sparse matrix)
    std::pair<int, int> key = {i, j};

    // find key in map
    auto iterator = smat.find(key);

    // output value in map
    // key in map -> return value
    if (iterator != smat.end()) { 
        smat.erase(iterator); 
    } 

}

SphereForceMomentStruct sphere_fms_fill(int num_particle)
{

    // initialize
    SphereForceMomentStruct sphere_fms;
    VectorDouble zeros_vecd(num_particle, 0.);
    VectorInt zeros_veci(num_particle, 0);

    // fill struct with zeros
    sphere_fms.num_particle = num_particle;
    sphere_fms.force_sum_x_vec = zeros_vecd;
    sphere_fms.force_sum_y_vec = zeros_vecd;
    sphere_fms.force_sum_z_vec = zeros_vecd;
    sphere_fms.moment_sum_x_vec = zeros_vecd;
    sphere_fms.moment_sum_y_vec = zeros_vecd;
    sphere_fms.moment_sum_z_vec = zeros_vecd;
    sphere_fms.force_average_x_vec = zeros_vecd;
    sphere_fms.force_average_y_vec = zeros_vecd;
    sphere_fms.force_average_z_vec = zeros_vecd;
    sphere_fms.moment_average_x_vec = zeros_vecd;
    sphere_fms.moment_average_y_vec = zeros_vecd;
    sphere_fms.moment_average_z_vec = zeros_vecd;
    sphere_fms.num_contact_vec = zeros_veci;

    return sphere_fms;

}

#endif
