#ifndef CONTAINER_SMAT_INTEGRABLE
#define CONTAINER_SMAT_INTEGRABLE
#include <map>
#include <vector>
#include "container_typedef.hpp"

struct SparseMatrixIntegrable
{
    SparseMatrixDouble u;
    SparseMatrixDouble dudt;
};

#endif
