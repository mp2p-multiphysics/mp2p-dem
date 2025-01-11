#ifndef PARTICLE_SPHERE
#define PARTICLE_SPHERE
#include "container_typedef.hpp"

namespace DEM
{

class ParticleSphere
{

    public:

    // number of particles
    int num_element = 0;
    int num_element_max = 0;

    // id of particles
    // pid - permanent ID; remains the same even after particles are deleted
    // tid - temporary ID; index of particles
    VectorInt tid_to_pid_vec;
    MapIntInt pid_to_tid_map;

    // materials
    VectorInt material_vec;

    // positions and velocities
    std::vector<EigenVector3D> position_vec;
    std::vector<EigenVector3D> velocity_vec;
    std::vector<EigenVector3D> angularposition_vec;
    std::vector<EigenVector3D> angularvelocity_vec;

    // geometries
    VectorDouble radius_vec;

    // forces
    std::vector<EigenVector3D> force_vec;
    std::vector<EigenVector3D> moment_vec;

    // default constructor
    ParticleSphere() {}

    private:

};

}

#endif
