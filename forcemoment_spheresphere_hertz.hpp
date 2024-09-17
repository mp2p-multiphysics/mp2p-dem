#ifndef FORCEMOMENT_SPHERESPHERE_HERTZ
#define FORCEMOMENT_SPHERESPHERE_HERTZ
#include <map>
#include <utility>
#include <vector>
#include "collisioncheck_spheresphere_naive.hpp"
#include "collisioncheck_spheresphere_sweep_1dx.hpp"
#include "collisioncheck_spheresphere_sweep_1dy.hpp"
#include "collisioncheck_spheresphere_sweep_1dz.hpp"
#include "container_smat_integrable.hpp"
#include "container_sphere.hpp"
#include "container_typedef.hpp"

template <class CollisionCheckSphereSphere>
class ForceMomentSphereSphereHertz
{
    /*

    Calculates collision forces and moments between spheres.
    Uses the Hertzian (spring-dashpot) model to calculate collision forces.

    Variables
    =========
    radius_vec_in : VectorDouble
        vector with the radius of each type of sphere.
    springconstant_normal_mat_in : MatrixDouble
        Matrix (nested vector) with the normal spring constant of each type of sphere-sphere interaction.
    springconstant_tangent_mat_in : MatrixDouble
        Matrix with the tangential spring constant of each type of sphere-sphere interaction.
    dampingcoefficient_normal_mat_in : MatrixDouble
        Matrix with the normal damping coefficient of each type of sphere-sphere interaction.
    dampingcoefficient_tangent_mat_in : MatrixDouble
        Matrix with the tangential damping coefficient of each type of sphere-sphere interaction.
    frictioncoefficient_sliding_mat_in : MatrixDouble
        Matrix with the sliding friction coefficient of each type of sphere-sphere interaction.
    frictioncoefficient_rolling_mat_in : MatrixDouble
        Matrix with the rolling friction coefficient of each type of sphere-sphere interaction.

    Functions
    =========
    add_forcemoment : void
        Adds forces and moments to spheres in the simulation.

    */

    public:

    // variables
    VectorDouble radius_vec;
    MatrixDouble springconstant_normal_mat;
    MatrixDouble springconstant_tangent_mat;
    MatrixDouble dampingcoefficient_normal_mat;
    MatrixDouble dampingcoefficient_tangent_mat;
    MatrixDouble frictioncoefficient_sliding_mat;
    MatrixDouble frictioncoefficient_rolling_mat;

    // collision checker
    CollisionCheckSphereSphere collision_check;

    // functions
    void add_forcemoment(
        SphereForceMomentStruct &sphere_fms,
        SparseMatrixIntegrable &overlap_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs
    );

    // default constructor
    ForceMomentSphereSphereHertz()
    {

    }

    // constructor
    ForceMomentSphereSphereHertz(
        VectorDouble radius_vec_in,
        MatrixDouble springconstant_normal_mat_in,
        MatrixDouble springconstant_tangent_mat_in,
        MatrixDouble dampingcoefficient_normal_mat_in,
        MatrixDouble dampingcoefficient_tangent_mat_in,
        MatrixDouble frictioncoefficient_sliding_mat_in,
        MatrixDouble frictioncoefficient_rolling_mat_in
    )
    {
        
        // store variables
        radius_vec = radius_vec_in;
        springconstant_normal_mat = springconstant_normal_mat_in;
        springconstant_tangent_mat = springconstant_tangent_mat_in;
        dampingcoefficient_normal_mat = dampingcoefficient_normal_mat_in;
        dampingcoefficient_tangent_mat = dampingcoefficient_tangent_mat_in;
        frictioncoefficient_sliding_mat = frictioncoefficient_sliding_mat_in;
        frictioncoefficient_rolling_mat = frictioncoefficient_rolling_mat_in;

        // initialize collision checker
        collision_check.set_input(radius_vec);

    }

    private:

    // functions
    void calculate_forcemoment(
        SphereForceMomentStruct &sphere_fms,
        SparseMatrixIntegrable &overlap_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs,
        int indx_i, int indx_j
    );

};

template <class CollisionCheckSphereSphere>
void ForceMomentSphereSphereHertz<CollisionCheckSphereSphere>::add_forcemoment(
    SphereForceMomentStruct &sphere_fms,
    SparseMatrixIntegrable &overlap_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs
)
{
    /*

    Adds forces and moments to spheres in the simulation.

    Arguments
    =========
    sphere_fms : SphereForceMomentStruct
        struct with forces and moments on each sphere.
    overlap_tangent_mat : SparseMatrixIntegrable
        Sparse matrix with tangential overlaps between colliding spheres.
    sphere_pvs : SphereParticleVelocityStruct
        struct with position and velocity of each sphere.      

    Returns
    =======
    (none)

    */

    // generate preliminary list of collision pairs
    VectorPairInt collision_vec = collision_check.broad_search(sphere_pvs);

    // calculate force for each collision pair
    for (auto &collision_pair : collision_vec)
    {
        
        // get indices of particles
        int indx_i = collision_pair.first;
        int indx_j = collision_pair.second;

        // calculate forces
        calculate_forcemoment(sphere_fms, overlap_tangent_smat, sphere_pvs, indx_i, indx_j);
        
    }

}

template <class CollisionCheckSphereSphere>
void ForceMomentSphereSphereHertz<CollisionCheckSphereSphere>::calculate_forcemoment(
    SphereForceMomentStruct &sphere_fms,
    SparseMatrixIntegrable &overlap_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs,
    int indx_i, int indx_j
)
{
    /*

    Calculates forces and moments of colliding spheres.

    */

    // get particle id
    int id_i = sphere_pvs.id_vec[indx_i];
    int id_j = sphere_pvs.id_vec[indx_j];

    // get particle type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_j = sphere_pvs.type_vec[indx_j];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];
    double pos_x_j = sphere_pvs.position_x_vec[indx_j];
    double pos_y_j = sphere_pvs.position_y_vec[indx_j];
    double pos_z_j = sphere_pvs.position_z_vec[indx_j];

    // get particle radii
    double radius_i = radius_vec[type_i];
    double radius_j = radius_vec[type_j];

    // calculate displacement from center of i to j
    double delta_pos_x_ij = -pos_x_i + pos_x_j;
    double delta_pos_y_ij = -pos_y_i + pos_y_j;
    double delta_pos_z_ij = -pos_z_i + pos_z_j;

    // calculate distance from center of i to j
    double delta_pos_ij_mag = sqrt(delta_pos_x_ij*delta_pos_x_ij + delta_pos_y_ij*delta_pos_y_ij + delta_pos_z_ij*delta_pos_z_ij);

    // calculate normal overlap
    double overlap_normal_ij_val = -delta_pos_ij_mag + radius_i + radius_j;

    // skip if no collision
    // negative or zero normal overlap
    if (overlap_normal_ij_val <= 0)
    {
        smat_prune(overlap_tangent_smat.u, id_i, id_j);  // reset tangential overlap
        smat_prune(overlap_tangent_smat.dudt, id_i, id_j);
        return;
    }

    // get particle velocities
    double vel_x_i = sphere_pvs.velocity_x_vec[indx_i];
    double vel_y_i = sphere_pvs.velocity_y_vec[indx_i];
    double vel_z_i = sphere_pvs.velocity_z_vec[indx_i];
    double vel_x_j = sphere_pvs.velocity_x_vec[indx_j];
    double vel_y_j = sphere_pvs.velocity_y_vec[indx_j];
    double vel_z_j = sphere_pvs.velocity_z_vec[indx_j];

    // get particle angular velocities
    double angvel_x_i = sphere_pvs.angularvelocity_x_vec[indx_i];
    double angvel_y_i = sphere_pvs.angularvelocity_y_vec[indx_i];
    double angvel_z_i = sphere_pvs.angularvelocity_z_vec[indx_i];
    double angvel_x_j = sphere_pvs.angularvelocity_x_vec[indx_j];
    double angvel_y_j = sphere_pvs.angularvelocity_y_vec[indx_j];
    double angvel_z_j = sphere_pvs.angularvelocity_z_vec[indx_j];

    // get collision properties
    double springconstant_normal = springconstant_normal_mat[type_i][type_j];
    double springconstant_tangent = springconstant_tangent_mat[type_i][type_j];
    double dampingcoefficient_normal = dampingcoefficient_normal_mat[type_i][type_j];
    double dampingcoefficient_tangent = dampingcoefficient_tangent_mat[type_i][type_j];
    double frictioncoefficient_sliding = frictioncoefficient_sliding_mat[type_i][type_j];
    double frictioncoefficient_rolling = frictioncoefficient_rolling_mat[type_i][type_j];

    // get tangential overlap
    double overlap_tangent_ij_val = smat_get_value(overlap_tangent_smat.u, id_i, id_j);

    // calculate normal vector
    double helpvar_01 = 1./delta_pos_ij_mag;
    double normal_x_ij = delta_pos_x_ij*helpvar_01;
    double normal_y_ij = delta_pos_y_ij*helpvar_01;
    double normal_z_ij = delta_pos_z_ij*helpvar_01;

    // calculate relative velocity
    double relvel_x_ij = -normal_y_ij*(angvel_z_i*radius_i + angvel_z_j*radius_j) + normal_z_ij*(angvel_y_i*radius_i + angvel_y_j*radius_j) + vel_x_i - vel_x_j;
    double relvel_y_ij =  normal_x_ij*(angvel_z_i*radius_i + angvel_z_j*radius_j) - normal_z_ij*(angvel_x_i*radius_i + angvel_x_j*radius_j) + vel_y_i - vel_y_j;
    double relvel_z_ij = -normal_x_ij*(angvel_y_i*radius_i + angvel_y_j*radius_j) + normal_y_ij*(angvel_x_i*radius_i + angvel_x_j*radius_j) + vel_z_i - vel_z_j;

    // calculate magnitude of normal component of relative velocity
    double relvel_normal_ij_val = normal_x_ij*vel_x_i - normal_x_ij*vel_x_j + normal_y_ij*vel_y_i - normal_y_ij*vel_y_j + normal_z_ij*vel_z_i - normal_z_ij*vel_z_j;

    // calculate normal component of relative velocity
    double relvel_normal_x_ij = normal_x_ij*relvel_normal_ij_val;
    double relvel_normal_y_ij = normal_y_ij*relvel_normal_ij_val;
    double relvel_normal_z_ij = normal_z_ij*relvel_normal_ij_val;

    // calculate tangential component of relative velocity
    double relvel_tangent_x_ij = -relvel_normal_x_ij + relvel_x_ij;
    double relvel_tangent_y_ij = -relvel_normal_y_ij + relvel_y_ij;
    double relvel_tangent_z_ij = -relvel_normal_z_ij + relvel_z_ij;

    // calculate tangential vector
    // tangential vector is zero if tangential component of relative velocity is zero
    double helpvar_02 = 1./sqrt(relvel_tangent_x_ij*relvel_tangent_x_ij + relvel_tangent_y_ij*relvel_tangent_y_ij + relvel_tangent_z_ij*relvel_tangent_z_ij);
    double tangent_x_ij = relvel_tangent_x_ij*helpvar_02;
    double tangent_y_ij = relvel_tangent_y_ij*helpvar_02;
    double tangent_z_ij = relvel_tangent_z_ij*helpvar_02;
    if (relvel_tangent_x_ij == 0. && relvel_tangent_y_ij == 0. && relvel_tangent_z_ij == 0.)
    {
        tangent_x_ij = 0.;
        tangent_y_ij = 0.;
        tangent_z_ij = 0.;
    }

    // calculate tangential component of relative velocity
    double relvel_tangent_ij_val = relvel_x_ij*tangent_x_ij + relvel_y_ij*tangent_y_ij + relvel_z_ij*tangent_z_ij;

    // calculate magnitude of normal component of collision force
    double force_collision_normal_ij_val = -dampingcoefficient_normal*relvel_normal_ij_val - overlap_normal_ij_val*springconstant_normal;
    double force_collision_normal_ij_mag = abs(force_collision_normal_ij_val);

    // calculate normal component of collision force
    double force_collision_normal_x_ij = force_collision_normal_ij_val*normal_x_ij;
    double force_collision_normal_y_ij = force_collision_normal_ij_val*normal_y_ij;
    double force_collision_normal_z_ij = force_collision_normal_ij_val*normal_z_ij;

    // calculate magnitude of tangential component of collision force
    double force_collision_tangent_ij_val = -dampingcoefficient_tangent*relvel_tangent_ij_val - overlap_tangent_ij_val*springconstant_tangent;
    double force_collision_tangent_ij_mag = abs(force_collision_tangent_ij_val);

    // recalculate tangential force if it exceeds maximum static friction
    if (force_collision_tangent_ij_mag > frictioncoefficient_sliding*force_collision_normal_ij_mag)
    {
        
        // calcualte signum of tangential overlap
        double signum_overlap_tangent_ij_val = 1.;
        if (overlap_tangent_ij_val < 0.)
        {
            signum_overlap_tangent_ij_val = -1.;
        }
        
        // calculate magnitude of tangential component of collision force (friction)
        force_collision_tangent_ij_val = -force_collision_normal_ij_mag*frictioncoefficient_sliding*signum_overlap_tangent_ij_val;

    }

    // calculate tangential component of collision force
    double force_collision_tangent_x_ij = force_collision_tangent_ij_val*tangent_x_ij;
    double force_collision_tangent_y_ij = force_collision_tangent_ij_val*tangent_y_ij;
    double force_collision_tangent_z_ij = force_collision_tangent_ij_val*tangent_z_ij;

    // calculate collision force
    double force_collision_x_ij = force_collision_normal_x_ij + force_collision_tangent_x_ij;
    double force_collision_y_ij = force_collision_normal_y_ij + force_collision_tangent_y_ij;
    double force_collision_z_ij = force_collision_normal_z_ij + force_collision_tangent_z_ij;

    // update collision matrix
    smat_set_value(overlap_tangent_smat.dudt, id_i, id_j, relvel_tangent_ij_val);
    
    // add forces on i
    sphere_fms.force_sum_x_vec[indx_i] += force_collision_x_ij;
    sphere_fms.force_sum_y_vec[indx_i] += force_collision_y_ij;
    sphere_fms.force_sum_z_vec[indx_i] += force_collision_z_ij;

    // apply Newton's third law to get forces on j
    sphere_fms.force_sum_x_vec[indx_j] += (-force_collision_x_ij);
    sphere_fms.force_sum_y_vec[indx_j] += (-force_collision_y_ij);
    sphere_fms.force_sum_z_vec[indx_j] += (-force_collision_z_ij);

    // calculate collision moment on i
    double moment_collision_x_ij = radius_i*(-force_collision_tangent_y_ij*normal_z_ij + force_collision_tangent_z_ij*normal_y_ij);
    double moment_collision_y_ij = radius_i*( force_collision_tangent_x_ij*normal_z_ij - force_collision_tangent_z_ij*normal_x_ij);
    double moment_collision_z_ij = radius_i*(-force_collision_tangent_x_ij*normal_y_ij + force_collision_tangent_y_ij*normal_x_ij);

    // calculate collision moment on j
    // same direction for i and j since both collision force and normal vector switch signs
    double moment_collision_x_ji = radius_j*(-force_collision_tangent_y_ij*normal_z_ij + force_collision_tangent_z_ij*normal_y_ij);
    double moment_collision_y_ji = radius_j*( force_collision_tangent_x_ij*normal_z_ij - force_collision_tangent_z_ij*normal_x_ij);
    double moment_collision_z_ji = radius_j*(-force_collision_tangent_x_ij*normal_y_ij + force_collision_tangent_y_ij*normal_x_ij);

    // calculate relative angular velocities of i and j
    double relangvel_x_ij = -angvel_x_j + angvel_x_i;
    double relangvel_y_ij = -angvel_y_j + angvel_y_i;
    double relangvel_z_ij = -angvel_z_j + angvel_z_i;

    // calculate unit vector pointing to relative angular velocity
    // unit vector is zero if difference between angular velocities is zero
    double helpvar_03 = 1./sqrt(relangvel_x_ij*relangvel_x_ij + relangvel_y_ij*relangvel_y_ij + relangvel_z_ij*relangvel_z_ij);
    double axis_x_ij = relangvel_x_ij*helpvar_03;
    double axis_y_ij = relangvel_y_ij*helpvar_03;
    double axis_z_ij = relangvel_z_ij*helpvar_03;
    if (relangvel_x_ij == 0. && relangvel_y_ij == 0. && relangvel_z_ij == 0.)
    {
        axis_x_ij = 0.;
        axis_y_ij = 0.;
        axis_z_ij = 0.;
    }

    // calculate effective radius
    double radius_effective = radius_i*radius_j/(radius_i + radius_j);

    // calculate magnitude of fricion moment
    double moment_friction_ij_val = -force_collision_normal_ij_mag*frictioncoefficient_rolling*radius_effective;

    // calculate friction moment on i
    double moment_friction_x_ij = moment_friction_ij_val*axis_x_ij;
    double moment_friction_y_ij = moment_friction_ij_val*axis_y_ij;
    double moment_friction_z_ij = moment_friction_ij_val*axis_z_ij;

    // calculate friction moment on j
    // axis switches signs
    double moment_friction_x_ji = -moment_friction_ij_val*axis_x_ij;
    double moment_friction_y_ji = -moment_friction_ij_val*axis_y_ij;
    double moment_friction_z_ji = -moment_friction_ij_val*axis_z_ij;

    // add moments on i
    sphere_fms.moment_sum_x_vec[indx_i] += moment_collision_x_ij + moment_friction_x_ij;
    sphere_fms.moment_sum_y_vec[indx_i] += moment_collision_y_ij + moment_friction_y_ij;
    sphere_fms.moment_sum_z_vec[indx_i] += moment_collision_z_ij + moment_friction_z_ij;

    // add moments on j
    sphere_fms.moment_sum_x_vec[indx_j] += moment_collision_x_ji + moment_friction_x_ji;
    sphere_fms.moment_sum_y_vec[indx_j] += moment_collision_y_ji + moment_friction_y_ji;
    sphere_fms.moment_sum_z_vec[indx_j] += moment_collision_z_ji + moment_friction_z_ji;

}

#endif
