#ifndef INSERTDELETE_BASE
#define INSERTDELETE_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class InsertDeleteBase
{
    /*

    Base class for insertion or deletion of group objects.

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
    InsertDeleteBase() {}

    private:

};

}

#endif
