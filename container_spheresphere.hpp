#ifndef CONTAINER_SPHERESPHERE
#define CONTAINER_SPHERESPHERE
#include <map>
#include <utility>
#include <vector>
#include "container_spheresphere.hpp"
#include "particle_sphere.hpp"

namespace DEM
{

// collision pair typedef
typedef std::pair<ParticleSphere*, int> SpherePIDPair;
typedef std::pair<SpherePIDPair, SpherePIDPair> CollisionSphereSpherePair;
typedef std::vector<CollisionSphereSpherePair> CollisionSphereSphereVector;

// collision matrix
class CollisionSphereSphereMatrix
{

    public:

    // map of collision pair to value
    std::map<CollisionSphereSpherePair, double> value_map;

    // functions
    double get_value(CollisionSphereSpherePair collision_pair);
    void set_value(CollisionSphereSpherePair collision_pair, double value);
    void prune(CollisionSphereSpherePair collision_pair);

    // default constructor
    CollisionSphereSphereMatrix() {};

};

double CollisionSphereSphereMatrix::get_value(CollisionSphereSpherePair collision_pair)
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

void CollisionSphereSphereMatrix::set_value(CollisionSphereSpherePair collision_pair, double value)
{

    // assign value
    value_map[collision_pair] = value;

}

void CollisionSphereSphereMatrix::prune(CollisionSphereSpherePair collision_pair)
{

    // remove value
    // no effect if key is not in map
    value_map.erase(collision_pair);

}

}

#endif
