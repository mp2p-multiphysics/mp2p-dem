#ifndef PARAMETER_UNARY
#define PARAMETER_UNARY
#include "container_typedef.hpp"

namespace DEM
{

class ParameterUnary
{

    public:

    // values
    int num_value = 0;
    VectorDouble value_vec;

    // functions
    double get_value(int indx_i);

    // default constructor
    ParameterUnary() {}

    // constructor
    ParameterUnary(int num_value_in, VectorDouble value_vec_in)
    {

        // store inputs
        num_value = num_value_in;
        value_vec = value_vec_in;

    }

    private:

};

double ParameterUnary::get_value(int indx_i)
{
    return value_vec[indx_i];
}

}

#endif
