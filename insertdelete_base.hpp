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
    InsertDeleteBase() {}

    private:

};

}

#endif
