#ifndef PHYSICS_SPHERESPHERE_COLLISION_HERTZ
#define PHYSICS_SPHERESPHERE_COLLISION_HERTZ
#include "collider_spheresphere_base.hpp"
#include "parameter_1d.hpp"
#include "parameter_2d.hpp"
#include "physics_base.hpp"

namespace DEM
{

class PhysicsSphereSphereCollisionHertz : public PhysicsBase
{

    public:
    
    // collider
    ColliderSphereSphereBase* collider_ptr;
    CollisionSphereSphereVector collision_vec;

    // material parameters
    Parameter1D* density_ptr;
    Parameter2D* spring_normal_ptr;
    Parameter2D* spring_tangent_ptr;
    Parameter2D* damping_normal_ptr;
    Parameter2D* damping_tangent_ptr;
    Parameter2D* friction_sliding_ptr;
    Parameter2D* friction_rolling_ptr;

    // tangential overlap matrices
    CollisionSphereSphereMatrix overlap_tangent_mat;
    CollisionSphereSphereMatrix ddt_overlap_tangent_mat;

    // functions used in timestepping
    void compute_force(int ts);
    void compute_position_velocity(int ts) {};
    void compute_insert_delete(int ts) {};

    // default constructor
    PhysicsSphereSphereCollisionHertz() {};

    // constructor
    PhysicsSphereSphereCollisionHertz
    (
        ColliderSphereSphereBase &collider_in, Parameter1D &density_in,
        Parameter2D &spring_normal_in, Parameter2D &spring_tangent_in,
        Parameter2D &damping_normal_in, Parameter2D &damping_tangent_in,
        Parameter2D &friction_sliding_in, Parameter2D &friction_rolling_in
    )
    {

        // store collider
        collider_ptr = &collider_in;

        // store material parameters
        density_ptr = &density_in;
        spring_normal_ptr = &spring_normal_in;
        spring_tangent_ptr = &spring_tangent_in;
        damping_normal_ptr = &damping_normal_in;
        damping_tangent_ptr = &damping_tangent_in;
        friction_sliding_ptr = &friction_sliding_in;
        friction_rolling_ptr = &friction_rolling_in;

    }

    private:

    // function
    void compute_force_pair(CollisionSphereSpherePair collision_pair);

};

void PhysicsSphereSphereCollisionHertz::compute_force(int ts)
{

    // update collider list
    collider_ptr->compute_collide_pair(collision_vec, ts);

    // iterate through each colliding pair
    for (auto collision_pair : collision_vec)
    {
        compute_force_pair(collision_pair);
    }

}

void PhysicsSphereSphereCollisionHertz::compute_force_pair(CollisionSphereSpherePair collision_pair)
{

    // get sphere and permanent ID pairs
    SpherePIDPair sphere_pid_i = collision_pair.first;
    SpherePIDPair sphere_pid_j = collision_pair.second;
    
    // sphere i
    ParticleSphere* sphere_i_ptr = sphere_pid_i.first;
    int pid_i = sphere_pid_i.second;

    // sphere j
    ParticleSphere* sphere_j_ptr = sphere_pid_j.first;
    int pid_j = sphere_pid_j.second;

    // get temporary ID for array indexing
    int tid_i = sphere_i_ptr->pid_to_tid_map[pid_i];
    int tid_j = sphere_j_ptr->pid_to_tid_map[pid_j];

    // get particle positions
    EigenVector3D pos_i = sphere_i_ptr->position_vec[tid_i];
    EigenVector3D pos_j = sphere_j_ptr->position_vec[tid_j];

    // get particle radii
    double radius_i = sphere_i_ptr->radius_vec[tid_i];
    double radius_j = sphere_j_ptr->radius_vec[tid_j];

    // calculate displacement from center of i to j
    EigenVector3D delta_pos_ij = -pos_i + pos_j;
    double delta_pos_ij_mag = delta_pos_ij.norm();

    // calculate normal overlap
    double overlap_normal_ij_val = -delta_pos_ij_mag + radius_i + radius_j;

    // skip if no collision
    // negative or zero normal overlap
    if (overlap_normal_ij_val <= 0)
    {
        overlap_tangent_mat.prune(collision_pair);  // reset tangential overlap
        ddt_overlap_tangent_mat.prune(collision_pair);
        return;
    }

    // get particle velocities
    EigenVector3D vel_i = sphere_i_ptr->velocity_vec[tid_i];
    EigenVector3D vel_j = sphere_j_ptr->velocity_vec[tid_j];
    EigenVector3D angvel_i = sphere_i_ptr->angularvelocity_vec[tid_i];
    EigenVector3D angvel_j = sphere_j_ptr->angularvelocity_vec[tid_j];

    // get material type
    int material_i = sphere_i_ptr->material_vec[tid_i];
    int material_j = sphere_j_ptr->material_vec[tid_j];

    // get collision properties
    double spring_normal = spring_normal_ptr->get_value(material_i, material_j);
    double spring_tangent = spring_tangent_ptr->get_value(material_i, material_j);
    double damping_normal = damping_normal_ptr->get_value(material_i, material_j);
    double damping_tangent = damping_tangent_ptr->get_value(material_i, material_j);
    double friction_sliding = friction_sliding_ptr->get_value(material_i, material_j);
    double friction_rolling = friction_rolling_ptr->get_value(material_i, material_j);

    // get tangential overlap
    double overlap_tangent_ij_val = overlap_tangent_mat.get_value(collision_pair);

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

    // update collision matrix
    ddt_overlap_tangent_mat.set_value(collision_pair, relvel_tangent_ij_val);
    
    // add forces on sphere i
    // apply Newton's third law to get forces on j
    sphere_i_ptr->force_vec[tid_i] +=  force_collision_ij;
    sphere_j_ptr->force_vec[tid_j] += -force_collision_ij;

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
    sphere_i_ptr->moment_vec[tid_i] += moment_collision_ij + moment_friction_ij;
    sphere_j_ptr->moment_vec[tid_j] += moment_collision_ji + moment_friction_ji;

}

}

#endif
