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
    ModifyBase() {}

    private:

};

}

#endif
