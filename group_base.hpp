#ifndef GROUP_BASE
#define GROUP_BASE

namespace DEM
{

class GroupBase
{

    public:

    // functions
    virtual void write_output(int ts) {};
    virtual void clear_forcemoment() {};

    // default constructor
    GroupBase() {}

    private:

};

}

#endif
