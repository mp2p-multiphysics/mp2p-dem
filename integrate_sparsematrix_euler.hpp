#ifndef INTEGRATE_SPARSEMATRIX_MODIFIEDEULER
#define INTEGRATE_SPARSEMATRIX_MODIFIEDEULER
#include <vector>
#include "container_smat_integrable.hpp"
#include "container_typedef.hpp"

class IntegrateSparseMatrixEuler
{

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
