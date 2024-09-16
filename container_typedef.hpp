#ifndef CONTAINER_TYPEDEF
#define CONTAINER_TYPEDEF
#include <map>
#include <utility>
#include <vector>

typedef std::vector<double> VectorDouble;
typedef std::vector<int> VectorInt;

typedef std::vector<std::pair<double, double>> VectorPairDouble;
typedef std::vector<std::pair<int, int>> VectorPairInt;
typedef std::vector<std::pair<int, double>> VectorPairIntDouble;

typedef std::vector<std::vector<double>> MatrixDouble;
typedef std::vector<std::vector<int>> MatrixInt;

typedef std::map<std::pair<int,int>, double> SparseMatrixDouble;
typedef std::map<std::pair<int,int>, int> SparseMatrixInt;

double smat_get_value(SparseMatrixDouble &smat, int i, int j)
{

    // create key for map (sparse matrix)
    std::pair<int, int> key = {i, j};

    // find key in map
    auto iterator = smat.find(key);

    // output value in map
    // key in map -> return value
    if (iterator != smat.end())  
    {
        return iterator -> second;
    }

    // key not in map -> return 0
    return 0.;

}

void smat_set_value(SparseMatrixDouble &smat, int i, int j, double value)
{

    // create key for map (sparse matrix)
    std::pair<int, int> key = {i, j};

    // assign value to map
    smat[key] = value;

}

void smat_prune(SparseMatrixDouble &smat, int i, int j)
{

    // create key for map (sparse matrix)
    std::pair<int, int> key = {i, j};

    // find key in map
    auto iterator = smat.find(key);

    // output value in map
    // key in map -> return value
    if (iterator != smat.end()) { 
        smat.erase(iterator); 
    } 

}

#endif
