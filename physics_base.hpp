#ifndef PHYSICS_BASE
#define PHYSICS_BASE

namespace DEM
{

class PhysicsBase
{

    public:
    
    // functions used in timestepping
    void compute_force(int ts) {};
    void compute_position_velocity(int ts) {};
    void compute_insert_delete(int ts) {};

    // default constructor
    PhysicsBase() {}

    private:

};

}

#endif
