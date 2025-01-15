#ifndef CONTAINER_SPHEREMESH
#define CONTAINER_SPHEREMESH
#include <map>
#include <utility>
#include <vector>
#include "particle_sphere.hpp"
#include "wall_mesh.hpp"

namespace DEM
{

// collision pair typedef
typedef std::pair<ParticleSphere*, int> SpherePIDPair;
typedef std::pair<WallMesh*, int> MeshPIDPair;
typedef std::pair<SpherePIDPair, MeshPIDPair> CollisionSphereMeshPair;
typedef std::vector<CollisionSphereMeshPair> CollisionSphereMeshVector;

// collision matrix
class CollisionSphereMeshMatrix
{

    public:

    // map of collision pair to value
    std::map<CollisionSphereMeshPair, double> value_map;

    // functions
    double get_value(CollisionSphereMeshPair collision_pair);
    void set_value(CollisionSphereMeshPair collision_pair, double value);
    void prune(CollisionSphereMeshPair collision_pair);

    // default constructor
    CollisionSphereMeshMatrix() {};

};

double CollisionSphereMeshMatrix::get_value(CollisionSphereMeshPair collision_pair)
{

    // check if key is in map
    // otherwise return 0
    if (value_map.find(collision_pair) != value_map.end())
    {
        return value_map[collision_pair];
    }
    else
    {
        return 0.;
    }

}

void CollisionSphereMeshMatrix::set_value(CollisionSphereMeshPair collision_pair, double value)
{

    // assign value
    value_map[collision_pair] = value;

}

void CollisionSphereMeshMatrix::prune(CollisionSphereMeshPair collision_pair)
{

    // remove value
    // no effect if key is not in map
    value_map.erase(collision_pair);

}

}

#endif
