#ifndef INTEGRAL_BASE
#define INTEGRAL_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class IntegralBase
{
    /*

    Base class for position and velocity calculations.

    Functions
    =========
    update : void
        Updates this object.
    initialize : void
        Initializes this object.

    */

    public:

    // functions
    virtual void initialize(double dt_in) {};
    virtual void update(int ts) {};

    // default constructor
    IntegralBase() {}

    private:

};

}

#endif
