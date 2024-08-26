#ifndef INTEGRATE_SPARSEMATRIX_MODIFIEDEULER
#define INTEGRATE_SPARSEMATRIX_MODIFIEDEULER
#include <vector>
#include "container_typedef.hpp"

class IntegrateSparseMatrixEuler
{

    public:

    // variables
    double dt;

    // functions
    void update_with_time_derivative
    (
        SparseMatrixDouble &smat,
        SparseMatrixDouble &differential_smat_over_dt      
    );
    void update_with_smat_derivative
    (
        SparseMatrixDouble &smat,
        SparseMatrixDouble &differential_smat
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

void IntegrateSparseMatrixEuler::update_with_time_derivative
(
    SparseMatrixDouble &smat,
    SparseMatrixDouble &differential_smat_over_dt      
)
{

    // update sparse matrix
    for (auto &overlap_pair : differential_smat_over_dt)
    {

        // get key (particles) and value (differential overlap)
        std::pair<int, int> key = overlap_pair.first;
        double value = overlap_pair.second;

        // increment tangential overlap
        smat[key] += value * dt;

    }

}

void IntegrateSparseMatrixEuler::update_with_smat_derivative
(
    SparseMatrixDouble &smat,
    SparseMatrixDouble &differential_smat
)
{

    // update sparse matrix
    for (auto &overlap_pair : differential_smat)
    {

        // get key (particles) and value (differential overlap)
        std::pair<int, int> key = overlap_pair.first;
        double value = overlap_pair.second;

        // increment tangential overlap
        smat[key] += value;

    }

}

#endif
