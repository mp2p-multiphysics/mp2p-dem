#ifndef PARAMETER_UNARY
#define PARAMETER_UNARY
#include "container_typedef.hpp"

namespace DEM
{

class ParameterUnary
{
    /*

    Material properties.

    Variables
    =========
    num_value_in : int
        Number of materials
    value_vec_in : VectorDouble
        Vector of material properties.

    Functions
    =========
    get_value : double
        Returns properties of a material.

    */

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

double ParameterUnary::get_value(int mid_i)
{
    /*

    Returns properties of a material.

    Arguments
    =========
    mid_i : int
        Material ID of i.

    Returns
    =======
    value : double
        Property of the material.

    */
    
    return value_vec[mid_i];

}

}

#endif
