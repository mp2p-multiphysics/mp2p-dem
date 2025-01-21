#ifndef FORCEMOMENT_SPHERESPHERE_HERTZ
#define FORCEMOMENT_SPHERESPHERE_HERTZ
#include "collider_spheresphere_base.hpp"
#include "container_typedef.hpp"
#include "forcemoment_base.hpp"
#include "group_sphere.hpp"
#include "parameter_binary.hpp"

namespace DEM
{

class ForceMomentSphereSphereHertz : public ForceMomentBase
{

    public:

    // sphere group
    ColliderSphereSphereBase* collider_ptr;
    SphereGroup* spheregroup_ptr;

    // parameters
    ParameterBinary* spring_normal_ptr;
    ParameterBinary* spring_tangent_ptr;
    ParameterBinary* damping_normal_ptr;
    ParameterBinary* damping_tangent_ptr;
    ParameterBinary* friction_sliding_ptr;
    ParameterBinary* friction_rolling_ptr;

    // functions
    std::vector<BaseGroup*> get_group_ptr_vec() {return {spheregroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    ForceMomentSphereSphereHertz() {}

    // constructor
    ForceMomentSphereSphereHertz
    (
        SphereGroup &spheregroup_in, ColliderSphereSphereBase &collider_in,
        ParameterBinary &spring_normal_in, ParameterBinary &spring_tangent_in,
        ParameterBinary &damping_normal_in, ParameterBinary &damping_tangent_in,
        ParameterBinary &friction_sliding_in, ParameterBinary &friction_rolling_in
    )
    {

        // store sphere group
        collider_ptr = &collider_in;
        spheregroup_ptr = &spheregroup_in;

        // store parameters
        spring_normal_ptr = &spring_normal_in;
        spring_tangent_ptr = &spring_tangent_in;
        damping_normal_ptr = &damping_normal_in;
        damping_tangent_ptr = &damping_tangent_in;
        friction_sliding_ptr = &friction_sliding_in;
        friction_rolling_ptr = &friction_rolling_in;

    }

    private:

    // functions
    void compute_force_pair(int indx_i, int indx_j, double dt);

};

void ForceMomentSphereSphereHertz::update(int ts, double dt)
{

    // update vector of collision pairs
    collider_ptr->update_collision_vec();
    
    // iterate through each pair
    for (auto indx_pair : collider_ptr->get_collision_vec())
    {
        compute_force_pair(indx_pair.first, indx_pair.second, dt);
    }

}

void ForceMomentSphereSphereHertz::compute_force_pair(int indx_i, int indx_j, double dt)
{

    // get spheres
    Sphere sphere_i = spheregroup_ptr->sphere_vec[indx_i];
    Sphere sphere_j = spheregroup_ptr->sphere_vec[indx_j];

    // get particle positions
    EigenVector3D pos_i = sphere_i.position;
    EigenVector3D pos_j = sphere_j.position;

    // get particle radii
    double radius_i = sphere_i.radius;
    double radius_j = sphere_j.radius;

    // calculate displacement from center of i to j
    EigenVector3D delta_pos_ij = -pos_i + pos_j;
    double delta_pos_ij_mag = delta_pos_ij.norm();

    // calculate normal overlap
    double overlap_normal_ij_val = -delta_pos_ij_mag + radius_i + radius_j;

    // skip if no collision
    // negative or zero normal overlap
    if (overlap_normal_ij_val <= 0)
    {
//        overlap_tangent_mat.prune(collision_pair);  // reset tangential overlap
//        ddt_overlap_tangent_mat.prune(collision_pair);
        return;
    }

    // get particle velocities
    EigenVector3D vel_i = sphere_i.velocity;
    EigenVector3D vel_j = sphere_j.velocity;
    EigenVector3D angvel_i = sphere_i.angularvelocity;
    EigenVector3D angvel_j = sphere_j.angularvelocity;

    // get material type
    int mid_i = sphere_i.mid;
    int mid_j = sphere_j.mid;

    // get collision properties
    double spring_normal = spring_normal_ptr->get_value(mid_i, mid_j);
    double spring_tangent = spring_tangent_ptr->get_value(mid_i, mid_j);
    double damping_normal = damping_normal_ptr->get_value(mid_i, mid_j);
    double damping_tangent = damping_tangent_ptr->get_value(mid_i, mid_j);
    double friction_sliding = friction_sliding_ptr->get_value(mid_i, mid_j);
    double friction_rolling = friction_rolling_ptr->get_value(mid_i, mid_j);

//    // get tangential overlap
//    double overlap_tangent_ij_val = overlap_tangent_mat.get_value(collision_pair);
    double overlap_tangent_ij_val = 0.;

    // calculate normal vector
    EigenVector3D normal_ij = delta_pos_ij/delta_pos_ij_mag;

    // calculate relative velocity
    EigenVector3D relvel_ij = vel_i - vel_j + (radius_i*angvel_i + radius_j*angvel_j).cross(normal_ij);

    // calculate normal component of relative velocity
    double relvel_normal_ij_val = relvel_ij.dot(normal_ij);
    EigenVector3D relvel_normal_ij = relvel_normal_ij_val*normal_ij;

    // calculate tangential component of relative velocity
    EigenVector3D relvel_tangent_ij = relvel_ij - relvel_normal_ij;
    double relvel_tangent_ij_mag = relvel_tangent_ij.norm();

    // calculate tangential vector
    // tangential vector is zero if tangential component of relative velocity is zero
    EigenVector3D tangent_ij = relvel_tangent_ij/relvel_tangent_ij_mag;
    if (relvel_tangent_ij_mag < 1e-8)
    {
        tangent_ij = {0., 0., 0.};
    }

    // calculate tangential component of relative velocity
    double relvel_tangent_ij_val = relvel_ij.dot(tangent_ij);

    // calculate normal component of collision force
    double force_collision_normal_ij_val = -damping_normal*relvel_normal_ij_val - overlap_normal_ij_val*spring_normal;
    double force_collision_normal_ij_mag = abs(force_collision_normal_ij_val);
    EigenVector3D force_collision_normal_ij = force_collision_normal_ij_val*normal_ij;

    // calculate tangential component of collision force
    double force_collision_tangent_ij_val = -damping_tangent*relvel_tangent_ij_val - overlap_tangent_ij_val*spring_tangent;
    double force_collision_tangent_ij_mag = abs(force_collision_tangent_ij_val);

    // recalculate tangential force if it exceeds maximum static friction
    if (force_collision_tangent_ij_mag > friction_sliding*force_collision_normal_ij_mag)
    {
        
        // calcualte signum of tangential overlap
        double signum_overlap_tangent_ij_val = 1.;
        if (overlap_tangent_ij_val < 0.)
        {
            signum_overlap_tangent_ij_val = -1.;
        }
        
        // calculate magnitude of tangential component of collision force (friction)
        force_collision_tangent_ij_val = -force_collision_normal_ij_mag*friction_sliding*signum_overlap_tangent_ij_val;

    }

    // calculate tangential component of collision force
    EigenVector3D force_collision_tangent_ij = force_collision_tangent_ij_val*tangent_ij;

    // calculate collision force
    EigenVector3D force_collision_ij = force_collision_normal_ij + force_collision_tangent_ij;

//    // update collision matrix
//    ddt_overlap_tangent_mat.set_value(collision_pair, relvel_tangent_ij_val);
    
    // add forces on sphere i
    // apply Newton's third law to get forces on j
    spheregroup_ptr->sphere_vec[indx_i].force +=  force_collision_ij;
    spheregroup_ptr->sphere_vec[indx_j].force += -force_collision_ij;

    // calculate collision moment on i
    // same direction for i and j since both collision force and normal vector switch signs
    EigenVector3D moment_collision_ij = radius_i*normal_ij.cross(force_collision_tangent_ij);
    EigenVector3D moment_collision_ji = radius_j*normal_ij.cross(force_collision_tangent_ij);

    // calculate relative angular velocities of i and j
    EigenVector3D relangvel_ij = angvel_i - angvel_j;
    double relangvel_ij_mag = relangvel_ij.norm();

    // calculate unit vector pointing to relative angular velocity
    // unit vector is zero if difference between angular velocities is zero
    EigenVector3D axis_ij = relangvel_ij/relangvel_ij_mag;
    if (relangvel_ij_mag < 1e-8)
    {
        axis_ij = {0., 0., 0.};
    }

    // calculate effective radius
    double radius_effective = radius_i*radius_j/(radius_i + radius_j);

    // calculate magnitude of fricion moment
    double moment_friction_ij_val = -force_collision_normal_ij_mag*friction_rolling*radius_effective;

    // calculate friction moment on i
    // axis switches signs for j
    EigenVector3D moment_friction_ij = moment_friction_ij_val*axis_ij;
    EigenVector3D moment_friction_ji = -moment_friction_ij;

    // add moments on i and j
    spheregroup_ptr->sphere_vec[indx_i].moment += moment_collision_ij + moment_friction_ij;
    spheregroup_ptr->sphere_vec[indx_j].moment += moment_collision_ji + moment_friction_ji;

}

}

#endif
