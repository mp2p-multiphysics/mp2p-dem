#ifndef PARAMETER_2D
#define PARAMETER_2D
#include "container_typedef.hpp"

namespace DEM
{

class Parameter2D
{

    public:

    // input variables
    int size = 0;
    VectorDouble value_vec;

    // map with values
    // [i][j] -> value
    std::unordered_map<int, std::unordered_map<int, double>> value_map;

    // functions
    double get_value(int indx_i, int indx_j);

    // default constructor
    Parameter2D() {};

    // constructor
    Parameter2D(int size_in, VectorDouble value_vec_in)
    {

        // store inputs
        size = size_in;
        value_vec = value_vec_in;

        // create map with values
        fill_value_map();

    }

    private:

    // functions
    void fill_value_map();

};

double Parameter2D::get_value(int indx_i, int indx_j)
{

    // return value
    return value_map[indx_i][indx_j];

}

void Parameter2D::fill_value_map()
{

    // initialize index for value_vec
    int indx_v = 0;

    // iterate through combinations of values
    for (int indx_i = 0; indx_i < size; indx_i++){
    for (int indx_j = indx_i; indx_j < size; indx_j++){
        
        // assume symmetric parameters
        value_map[indx_i][indx_j] = value_vec[indx_v];
        value_map[indx_j][indx_i] = value_vec[indx_v];

        // increment counter
        indx_v++;

    }}

}

}

#endif
