#ifndef INSERTDELETE_SPHERE_INSERT_CSV
#define INSERTDELETE_SPHERE_INSERT_CSV
#include <fstream>
#include <sstream>
#include "insertdelete_base.hpp"
#include "sphere_group.hpp"

namespace DEM
{

class InsertDeleteSphereInsertCSV : public InsertDeleteBase
{

    public:

    // sphere group
    SphereGroup* spheregroup_ptr;

    // insertion parameters
    int ts_insert = 0;
    std::string file_in_str;
    double scale_factor = 0.;
    double enlarge_ratio = 0.;

    // functions
    void update(int ts, double dt);

    // default constructor
    InsertDeleteSphereInsertCSV() {}

    // constructor
    InsertDeleteSphereInsertCSV(SphereGroup &spheregroup_in, int ts_insert_in, std::string file_in_str_in, double scale_factor_in = 1., double enlarge_ratio_in = 1.05)
    {

        // store inputs
        spheregroup_ptr = &spheregroup_in;
        ts_insert = ts_insert_in;
        file_in_str = file_in_str_in;
        scale_factor = scale_factor_in;
        enlarge_ratio = enlarge_ratio_in;

    }

    private:

    void read_csv();

};

void InsertDeleteSphereInsertCSV::update(int ts, double dt)
{

    // skip if not insertion timestep
    if (ts != ts_insert)
    {
        return;
    }

    // read csv file
    read_csv();

    // update collision pairs


}

void InsertDeleteSphereInsertCSV::read_csv()
{

    // read file with particle positions and velocities
    std::ifstream file_in_stream(file_in_str);

    // initialize for iteration
    bool is_header = true;  // true while reading header
    std::string line_str;  // stores lines in files

    // iterate for each line in the file
    while (std::getline(file_in_stream, line_str))
    {

        // skip header
        if (is_header)
        {
            is_header = false; // not reading header
            continue;
        }

        // convert line string into stringstream
        std::stringstream line_stream(line_str);
    
        // initialize for iteration
        int value_num = 0;  // counts position of value
        std::string value_str;  // stores values in lines
        Sphere sphere_sub;  // shpere object

        // iterate through each value
        while (std::getline(line_stream, value_str, ','))
        {

            // store values in appropriate vector
            switch (value_num)
            {
                case 0: sphere_sub.gid = spheregroup_ptr->gid_max; break;
                case 1: sphere_sub.mid = std::stoi(value_str); break;
                case 2: sphere_sub.position.coeffRef(0) = scale_factor * std::stod(value_str); break;
                case 3: sphere_sub.position.coeffRef(1) = scale_factor * std::stod(value_str); break;
                case 4: sphere_sub.position.coeffRef(2) = scale_factor * std::stod(value_str); break;
                case 5: sphere_sub.velocity.coeffRef(0) = scale_factor * std::stod(value_str); break;
                case 6: sphere_sub.velocity.coeffRef(1) = scale_factor * std::stod(value_str); break;
                case 7: sphere_sub.velocity.coeffRef(2) = scale_factor * std::stod(value_str); break;
                case 8: sphere_sub.angularposition.coeffRef(0) = std::stod(value_str); break;
                case 9: sphere_sub.angularposition.coeffRef(1) = std::stod(value_str); break;
                case 10: sphere_sub.angularposition.coeffRef(2) = std::stod(value_str); break;
                case 11: sphere_sub.angularvelocity.coeffRef(0) = std::stod(value_str); break;
                case 12: sphere_sub.angularvelocity.coeffRef(1) = std::stod(value_str); break;
                case 13: sphere_sub.angularvelocity.coeffRef(2) = std::stod(value_str); break;
                case 14:
                    sphere_sub.radius = scale_factor * std::stod(value_str);
                    sphere_sub.radius_enlarged = scale_factor * enlarge_ratio * std::stod(value_str);
                break;
            }

            // increment value count
            value_num++;

        }
    
        // insert spheres
        spheregroup_ptr->sphere_vec.push_back(sphere_sub);
        spheregroup_ptr->gid_max += 1;

    }

    // close file
    file_in_stream.close();

}

}

#endif
