#ifndef FORCEMOMENT_BASE
#define FORCEMOMENT_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class ForceMomentBase
{
    /*

    Base class for force and/or moment calculations.

    Functions
    =========
    initialize : void
        Initializes this object.
    update : void
        Updates this object.

    */

    public:

    // functions
    virtual void initialize(double dt) {};
    virtual void update(int ts) {};

    // default constructor
    ForceMomentBase() {}

    private:

};

}

#endif
