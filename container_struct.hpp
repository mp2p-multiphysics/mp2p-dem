#ifndef CONTAINER_STRUCT
#define CONTAINER_STRUCT
#include <map>
#include <vector>

struct TimeIntegratedSparseMatrix
{
    std::map<std::vector<int>, double> value_smat;
    std::map<std::vector<int>, double> differential_value_smat;
};

#endif
