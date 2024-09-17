#ifndef INTEGRATE_SPARSEMATRIX_MODIFIEDEULER
#define INTEGRATE_SPARSEMATRIX_MODIFIEDEULER
#include <vector>
#include "container_smat_integrable.hpp"
#include "container_typedef.hpp"

class IntegrateSparseMatrixEuler
{
    /*

    Integrates the values in a sparse matrix.
    Uses the Euler method.

    Variables
    =========
    dt_in : double
        Length of time step.

    Functions
    =========
    integrate_u : void
        Integrates the values in a sparse matrix.

    */

    public:

    // variables
    double dt;

    // functions
    void integrate_u
    (
        SparseMatrixIntegrable &smat
    );

    // default constructor
    IntegrateSparseMatrixEuler()
    {

    }

    // constructor
    IntegrateSparseMatrixEuler(double dt_in)
    {
        dt = dt_in;
    }

};

void IntegrateSparseMatrixEuler::integrate_u
(
    SparseMatrixIntegrable &smat  
)
{
    /*

    Integrates the values in a sparse matrix.
    Uses the Euler method.

    Arguments
    =========
    smat : SparseMatrixIntegrable
        Sparse matrix to be integrated.

    Returns
    =======
    (none)

    */

    // update sparse matrix
    for (auto &overlap_pair : smat.dudt)
    {

        // get key (particles) and value (differential overlap)
        std::pair<int, int> key = overlap_pair.first;
        double value = overlap_pair.second;

        // increment tangential overlap
        smat.u[key] += value * dt;

    }

}

#endif
