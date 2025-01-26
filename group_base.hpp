#ifndef GROUP_BASE
#define GROUP_BASE

namespace DEM
{

class BaseGroup
{
    /*

    Base class for group or collection of objects.

    Functions
    =========
    output_file : void
        Writes output files.

    */

    public:

    // functions
    virtual void output_file(int ts) {};

    // default constructor
    BaseGroup() {}

    private:

};

}

#endif
