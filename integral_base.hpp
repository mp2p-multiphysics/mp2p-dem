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
    get_group_ptr_vec : vector<BaseGroup*>
        Returns pointers to group objects affected by this object.
    update : void
        Updates this object.

    */

    public:

    // functions
    virtual std::vector<BaseGroup*> get_group_ptr_vec() {return {};};
    virtual void update(int ts, double dt) {};

    // default constructor
    IntegralBase() {}

    private:

};

}

#endif
