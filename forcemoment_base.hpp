#ifndef FORCEMOMENT_BASE
#define FORCEMOMENT_BASE

namespace DEM
{

class ForceMomentBase
{

    public:

    // functions
    virtual void update(int ts, double dt) {};

    // default constructor
    ForceMomentBase() {}

    private:

};

}

#endif
