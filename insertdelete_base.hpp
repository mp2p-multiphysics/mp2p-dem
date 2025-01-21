#ifndef INSERTDELETE_BASE
#define INSERTDELETE_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class InsertDeleteBase
{

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
