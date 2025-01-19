#ifndef INSERTDELETE_BASE
#define INSERTDELETE_BASE

namespace DEM
{

class InsertDeleteBase
{

    public:

    // functions
    virtual void update(int ts, double dt) {};

    // default constructor
    InsertDeleteBase() {}

    private:

};

}

#endif
