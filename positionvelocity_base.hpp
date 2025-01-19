#ifndef POSITIONVELOCITY_BASE
#define POSITIONVELOCITY_BASE

namespace DEM
{

class PositionVelocityBase
{

    public:

    // functions
    virtual void update(int ts, double dt) {};

    // default constructor
    PositionVelocityBase() {}

    private:

};

}

#endif
