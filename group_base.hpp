#ifndef GROUP_BASE
#define GROUP_BASE

namespace DEM
{

class BaseGroup
{

    public:

    // functions
    virtual void write_output(int ts) {};
    virtual void clear_forcemoment() {};

    // default constructor
    BaseGroup() {}

    private:

};

}

#endif
