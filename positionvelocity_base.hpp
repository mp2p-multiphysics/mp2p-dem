#ifndef POSITIONVELOCITY_BASE
#define POSITIONVELOCITY_BASE
#include <vector>
#include "group_base.hpp"

namespace DEM
{

class PositionVelocityBase
{

    public:

    // functions
    virtual std::vector<BaseGroup*> get_group_ptr_vec() {return {};};
    virtual void update(int ts, double dt) {};

    // default constructor
    PositionVelocityBase() {}

    private:

};

}

#endif
