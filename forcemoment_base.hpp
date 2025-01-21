#ifndef FORCEMOMENT_BASE
#define FORCEMOMENT_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class ForceMomentBase
{

    public:

    // functions
    virtual std::vector<BaseGroup*> get_group_ptr_vec() {return {};};
    virtual void update(int ts, double dt) {};

    // default constructor
    ForceMomentBase() {}

    private:

};

}

#endif
