#ifndef PARAMETER_BINARY
#define PARAMETER_BINARY
#include "container_typedef.hpp"

namespace DEM
{

class ParameterBinary
{
    /*

    Pairwise material properties.

    Variables
    =========
    num_value_in : int
        Number of materials
    value_vec_in : VectorDouble
        Vector of pairwise material properties.

    Functions
    =========
    get_value : double
        Returns properties for a pair of materials.

    Notes
    =====
    Properties of materials 0, 1, 2, ..., N in value_vec_in must be arranged as follows:
    (0, 0) (0, 1) (0, 2) (0, N) (1, 1) (1, 2) (1, N) (2, 2) (2, N) ...

    */

    public:

    // values
    int num_value = 0;
    VectorDouble value_vec;
    VectorDouble2D value_reshape_vec;

    // functions
    double get_value(int indx_i, int indx_j);

    // default constructor
    ParameterBinary() {}

    // constructor
    ParameterBinary(int num_value_in, VectorDouble value_vec_in)
    {

        // store inputs
        num_value = num_value_in;
        value_vec = value_vec_in;

        // reshape to 2D
        if (value_vec.size() == (num_value*(num_value + 1))/2)
        {
            reshape_triangle();
        }
        else if (value_vec.size() == num_value*num_value)
        {
            reshape_square();
        }

    }

    private:

    // functions
    void reshape_triangle();
    void reshape_square();

};

double ParameterBinary::get_value(int mid_i, int mid_j)
{
    /*

    Returns properties for a pair of materials.

    Arguments
    =========
    mid_i : int
        Material ID of i.
    mid_j : int
        Material ID of j.

    Returns
    =======
    value : double
        Property for the pair of materials.

    */
    
    return value_reshape_vec[mid_i][mid_j];

}

void ParameterBinary::reshape_triangle()
{

    // pre-allocate vector
    value_reshape_vec = VectorDouble2D(num_value, VectorDouble(num_value, 0.));

    // initialize
    int indx_k = 0;

    // assume that value_vec has entries arranged as follows:
    // (0, 0) (0, 1) (0, 2) (0, N) (1, 1) (1, 2) (1, N) ...
    for (int indx_i = 0; indx_i < num_value; indx_i++){
    for (int indx_j = indx_i; indx_j < num_value; indx_j++){
        value_reshape_vec[indx_i][indx_j] = value_vec[indx_k];
        value_reshape_vec[indx_j][indx_i] = value_vec[indx_k];
        indx_k++;
    }}

}

void ParameterBinary::reshape_square()
{

    // pre-allocate vector
    value_reshape_vec = VectorDouble2D(num_value, VectorDouble(num_value, 0.));

    // initialize
    int indx_k = 0;

    // assume that value_vec has entries arranged as follows:
    // (0, 0) (0, 1) (0, 2) (0, N) (1, 0) (1, 1) (1, 2) (1, N) ...
    for (int indx_i = 0; indx_i < num_value; indx_i++){
    for (int indx_j = 0; indx_j < num_value; indx_j++){
        value_reshape_vec[indx_i][indx_j] = value_vec[indx_k];
        indx_k++;
    }}

}

}

#endif
