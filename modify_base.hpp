#ifndef MODIFY_BASE
#define MODIFY_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class ModifyBase
{

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
