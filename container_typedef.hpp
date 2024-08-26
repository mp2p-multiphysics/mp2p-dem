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

#endif
