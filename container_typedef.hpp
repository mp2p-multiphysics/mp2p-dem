#ifndef CONTAINER_TYPEDEF
#define CONTAINER_TYPEDEF
#include <unordered_map>
#include <vector>
#include "Eigen/Eigen"

namespace DEM
{

// nested int vectors
typedef std::vector<int> VectorInt;
typedef std::vector<VectorInt> VectorInt2D;

// nested double vectors
typedef std::vector<double> VectorDouble;
typedef std::vector<VectorDouble> VectorDouble2D;

// unordered maps
typedef std::unordered_map<int, int> MapIntInt;

// Eigen objects
typedef Eigen::Vector3d EigenVector3D;

}

#endif
