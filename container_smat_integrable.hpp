#ifndef CONTAINER_SMAT_INTEGRABLE
#define CONTAINER_SMAT_INTEGRABLE
#include <map>
#include <vector>
#include "container_typedef.hpp"

struct SparseMatrixIntegrable
{
    /*
    
    Sparse matrices whose value is to be integrated with time.

    Variables
    =========
    u : SparseMatrixDouble
        Sparse matrix with value.
    dudt : SparseMatrixDouble
        Sparse matrix with derivative of value with respect to time.

    Notes
    =====
    This struct is primarily used to store tangential overlap data.

    */
    
    SparseMatrixDouble u;
    SparseMatrixDouble dudt;

};

#endif
