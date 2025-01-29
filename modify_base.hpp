#ifndef MODIFY_BASE
#define MODIFY_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class ModifyBase
{
    /*

    Base class for modifying the variables of an object.

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
    ModifyBase() {}

    private:
    std::vector<BaseGroup*> group_ptr_vec;

};

}

#endif
