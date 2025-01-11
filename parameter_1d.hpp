#ifndef PARAMETER_1D
#define PARAMETER_1D
#include "container_typedef.hpp"

namespace DEM
{

class Parameter1D
{

    public:

    // input variables
    int size = 0;
    VectorDouble value_vec;

    // map with values
    // [i] -> value
    std::unordered_map<int, double> value_map;

    // default constructor
    Parameter1D() {};

    // constructor
    Parameter1D(int size_in, VectorDouble value_vec_in)
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

void Parameter1D::fill_value_map()
{

    // iterate through values
    for (int indx_i = 0; indx_i < size; indx_i++)
    {
        value_map[indx_i] = value_vec[indx_i];
    }

}

}

#endif
