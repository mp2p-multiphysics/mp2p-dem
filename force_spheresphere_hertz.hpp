#ifndef FORCE_SPHERESPHERE_HERTZ
#define FORCE_SPHERESPHERE_HERTZ
#include <map>
#include <utility>
#include <vector>
#include "collisioncheck_spheresphere_sweepprune.hpp"
#include "container_function.hpp"
#include "container_sphere.hpp"
#include "container_typedef.hpp"


class ForceSphereSphereHertz
{

    public:

    // variables
    VectorDouble radius_vec;
    MatrixDouble spring_constant_normal_mat;
    MatrixDouble spring_constant_tangent_mat;
    MatrixDouble damping_coefficient_normal_mat;
    MatrixDouble damping_coefficient_tangent_mat;
    MatrixDouble friction_coefficient_sliding_mat;
    MatrixDouble friction_coefficient_rolling_mat;

    // collision checker
    CollisionCheckSphereSphereSweepPrune collision_check;

    // functions
    void add_force_moment(
        SphereForceMomentStruct &sphere_fms,
        SparseMatrixDouble &overlap_tangent_smat,
        SparseMatrixDouble &relative_velocity_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs
    );

    // default constructor
    ForceSphereSphereHertz()
    {

    }

    // constructor
    ForceSphereSphereHertz(
        VectorDouble radius_vec_in,
        MatrixDouble spring_constant_normal_mat_in,
        MatrixDouble spring_constant_tangent_mat_in,
        MatrixDouble damping_coefficient_normal_mat_in,
        MatrixDouble damping_coefficient_tangent_mat_in,
        MatrixDouble friction_coefficient_sliding_mat_in,
        MatrixDouble friction_coefficient_rolling_mat_in
    )
    {
        
        // input parameters
        radius_vec = radius_vec_in;
        spring_constant_normal_mat = spring_constant_normal_mat_in;
        spring_constant_tangent_mat = spring_constant_tangent_mat_in;
        damping_coefficient_normal_mat = damping_coefficient_normal_mat_in;
        damping_coefficient_tangent_mat = damping_coefficient_tangent_mat_in;
        friction_coefficient_sliding_mat = friction_coefficient_sliding_mat_in;
        friction_coefficient_rolling_mat = friction_coefficient_rolling_mat_in;

        // set collision checker
        collision_check.set_input_parameter(radius_vec);

    }

    private:

    // functions
    void calculate_force_moment(
        SphereForceMomentStruct &sphere_fms,
        SparseMatrixDouble &overlap_tangent_smat,
        SparseMatrixDouble &relative_velocity_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs,
        int indx_i, int indx_j
    );

};

void ForceSphereSphereHertz::add_force_moment(
    SphereForceMomentStruct &sphere_fms,
    SparseMatrixDouble &overlap_tangent_smat,
    SparseMatrixDouble &relative_velocity_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs
)
{

    // generate preliminary list of collision pairs
    VectorPairInt collision_vec = collision_check.broad_search(sphere_pvs);

    // calculate force for each collision pair
    for (auto &collision_pair : collision_vec)
    {
        
        // get indices of particles
        int indx_i = collision_pair.first;
        int indx_j = collision_pair.second;

        // calculate forces
        calculate_force_moment(sphere_fms, overlap_tangent_smat, relative_velocity_tangent_smat, sphere_pvs, indx_i, indx_j);
        
    }

}

void ForceSphereSphereHertz::calculate_force_moment(
    SphereForceMomentStruct &sphere_fms,
    SparseMatrixDouble &overlap_tangent_smat,
    SparseMatrixDouble &relative_velocity_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs,
    int indx_i, int indx_j
)
{

    // get particle id
    int id_i = sphere_pvs.id_vec[indx_i];
    int id_j = sphere_pvs.id_vec[indx_j];

    // get particle type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_j = sphere_pvs.type_vec[indx_j];

    // get particle radii
    double rad_i = radius_vec[type_i];
    double rad_j = radius_vec[type_j];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];
    double pos_x_j = sphere_pvs.position_x_vec[indx_j];
    double pos_y_j = sphere_pvs.position_y_vec[indx_j];
    double pos_z_j = sphere_pvs.position_z_vec[indx_j];

    // calculate normal overlap
    double dpos_ij_mag = sqrt((pos_x_i - pos_x_j)*(pos_x_i - pos_x_j) + (pos_y_i - pos_y_j)*(pos_y_i - pos_y_j) + (pos_z_i - pos_z_j)*(pos_z_i - pos_z_j));
    double overlap_norm_ij_mag = rad_i + rad_j - dpos_ij_mag;

    // skip if no collision
    // negative or zero normal overlap
    if (overlap_norm_ij_mag <= 0)
    {
        smat_prune(overlap_tangent_smat, id_i, id_j);  // reset tangential overlap
        smat_prune(relative_velocity_tangent_smat, id_i, id_j);
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

    // get tangential overlap
    double overlap_tang_ij_mag = smat_get_value(overlap_tangent_smat, id_i, id_j);

    // calculate normal vector
    double norm_x_ij = (-pos_x_i + pos_x_j)/dpos_ij_mag;
    double norm_y_ij = (-pos_y_i + pos_y_j)/dpos_ij_mag;
    double norm_z_ij = (-pos_z_i + pos_z_j)/dpos_ij_mag;

    // calculate relative velocity vector
    double relvel_x_ij = -norm_y_ij*(angvel_z_i*rad_i + angvel_z_j*rad_j) + norm_z_ij*(angvel_y_i*rad_i + angvel_y_j*rad_j) + vel_x_i - vel_x_j;
    double relvel_y_ij = norm_x_ij*(angvel_z_i*rad_i + angvel_z_j*rad_j) - norm_z_ij*(angvel_x_i*rad_i + angvel_x_j*rad_j) + vel_y_i - vel_y_j;
    double relvel_z_ij = -norm_x_ij*(angvel_y_i*rad_i + angvel_y_j*rad_j) + norm_y_ij*(angvel_x_i*rad_i + angvel_x_j*rad_j) + vel_z_i - vel_z_j;

    // calculate normal component of relative velocity vector
    double helpvar_01 = (relvel_x_ij*(pos_x_i - pos_x_j) + relvel_y_ij*(pos_y_i - pos_y_j) + relvel_z_ij*(pos_z_i - pos_z_j))/(dpos_ij_mag*dpos_ij_mag);
    double relvel_norm_x_ij = (pos_x_i - pos_x_j)*helpvar_01;
    double relvel_norm_y_ij = (pos_y_i - pos_y_j)*helpvar_01;
    double relvel_norm_z_ij = (pos_z_i - pos_z_j)*helpvar_01;

    // calculate tangential component of relative velocity vector
    double relvel_tang_x_ij = relvel_x_ij - relvel_norm_x_ij;
    double relvel_tang_y_ij = relvel_y_ij - relvel_norm_y_ij;
    double relvel_tang_z_ij = relvel_z_ij - relvel_norm_z_ij;

    // calculate tangential vector
    // tangential vector is zero if relative velocity (tangential component) is zero
    double helpvar_02 = 1./sqrt(relvel_tang_x_ij*relvel_tang_x_ij + relvel_tang_y_ij*relvel_tang_y_ij + relvel_tang_z_ij*relvel_tang_z_ij);
    double tang_x_ij = relvel_tang_x_ij*helpvar_02;
    double tang_y_ij = relvel_tang_y_ij*helpvar_02;
    double tang_z_ij = relvel_tang_z_ij*helpvar_02;
    if (relvel_tang_x_ij == 0. && relvel_tang_y_ij == 0. && relvel_tang_z_ij == 0.)
    {
        tang_x_ij = 0.;
        tang_y_ij = 0.;
        tang_z_ij = 0.;
    }

    // calculate normal component of relative velocity
    double relvel_norm_ij_mag = norm_x_ij*relvel_norm_x_ij + norm_y_ij*relvel_norm_y_ij + norm_z_ij*relvel_norm_z_ij;

    // calculate normal component of collision force
    double helpvar_03 = overlap_norm_ij_mag*spring_constant_normal_mat[type_i][type_j] + relvel_norm_ij_mag*damping_coefficient_normal_mat[type_i][type_j];
    double fce_coll_norm_x_ij = -norm_x_ij*helpvar_03;
    double fce_coll_norm_y_ij = -norm_y_ij*helpvar_03;
    double fce_coll_norm_z_ij = -norm_z_ij*helpvar_03;

    // calculate tangential component of relative velocity
    double relvel_tang_ij_mag = -tang_x_ij*(norm_y_ij*(angvel_z_i*rad_i + angvel_z_j*rad_j) - norm_z_ij*(angvel_y_i*rad_i + angvel_y_j*rad_j) - vel_x_i + vel_x_j) + tang_y_ij*(norm_x_ij*(angvel_z_i*rad_i + angvel_z_j*rad_j) - norm_z_ij*(angvel_x_i*rad_i + angvel_x_j*rad_j) + vel_y_i - vel_y_j) - tang_z_ij*(norm_x_ij*(angvel_y_i*rad_i + angvel_y_j*rad_j) - norm_y_ij*(angvel_x_i*rad_i + angvel_x_j*rad_j) - vel_z_i + vel_z_j);
    
    // calculate tangential component of collision force (hertzian contact)
    double helpvar_04 = overlap_tang_ij_mag*spring_constant_tangent_mat[type_i][type_j] + relvel_tang_ij_mag*damping_coefficient_tangent_mat[type_i][type_j];
    double fce_coll_tang_x_ij = -tang_x_ij*helpvar_04;
    double fce_coll_tang_y_ij = -tang_y_ij*helpvar_04;
    double fce_coll_tang_z_ij = -tang_z_ij*helpvar_04;

    // calculate magnitude of normal and tangential collision forces
    double fce_coll_norm_ij_mag = sqrt(fce_coll_norm_x_ij*fce_coll_norm_x_ij + fce_coll_norm_y_ij*fce_coll_norm_y_ij + fce_coll_norm_z_ij*fce_coll_norm_z_ij);
    double fce_coll_tang_ij_mag = sqrt(fce_coll_tang_x_ij*fce_coll_tang_x_ij + fce_coll_tang_y_ij*fce_coll_tang_y_ij + fce_coll_tang_z_ij*fce_coll_tang_z_ij);

    // recalculate tangential force if it exceeds maximum static friction
    if (fce_coll_tang_ij_mag > friction_coefficient_sliding_mat[type_i][type_j]*fce_coll_norm_ij_mag)
    {
        
        // calcualte signum of tangential overlap
        double sgn_overlap_tang_ij_mag = 1.;
        if (overlap_tang_ij_mag < 0.)
        {
            sgn_overlap_tang_ij_mag = -1.;
        }
        
        // calculate tangential component of collision force (friction)
        double helpvar_05 = friction_coefficient_sliding_mat[type_i][type_j]*sgn_overlap_tang_ij_mag*fce_coll_norm_ij_mag;
        fce_coll_tang_x_ij = -tang_x_ij*helpvar_05;
        fce_coll_tang_y_ij = -tang_y_ij*helpvar_05;
        fce_coll_tang_z_ij = -tang_z_ij*helpvar_05;

    }

    // calculate collision force
    double fce_coll_x_ij = fce_coll_norm_x_ij + fce_coll_tang_x_ij;
    double fce_coll_y_ij = fce_coll_norm_y_ij + fce_coll_tang_y_ij;
    double fce_coll_z_ij = fce_coll_norm_z_ij + fce_coll_tang_z_ij;

    // calculate collision moment
    double mom_coll_x_ij = rad_i*(-fce_coll_y_ij*norm_z_ij + fce_coll_z_ij*norm_y_ij);
    double mom_coll_y_ij = rad_i*(fce_coll_x_ij*norm_z_ij - fce_coll_z_ij*norm_x_ij);
    double mom_coll_z_ij = rad_i*(-fce_coll_x_ij*norm_y_ij + fce_coll_y_ij*norm_x_ij);

    // calculate unit vector pointing to relative angular velocity
    // unit vector is zero if relative angular velocity is zero
    double helpvar_06 = 1./sqrt((angvel_x_i - angvel_x_j)*(angvel_x_i - angvel_x_j) + (angvel_y_i - angvel_y_j)*(angvel_y_i - angvel_y_j) + (angvel_z_i - angvel_z_j)*(angvel_z_i - angvel_z_j));
    double unit_relangvel_x_ij = (angvel_x_i - angvel_x_j)*helpvar_06;
    double unit_relangvel_y_ij = (angvel_y_i - angvel_y_j)*helpvar_06;
    double unit_relangvel_z_ij = (angvel_z_i - angvel_z_j)*helpvar_06;
    if (angvel_x_i - angvel_x_j == 0. && angvel_y_i - angvel_y_j == 0. && angvel_z_i - angvel_z_j == 0.)
    {
        unit_relangvel_x_ij = 0.;
        unit_relangvel_y_ij = 0.;
        unit_relangvel_z_ij = 0.;
    }

    // calculate friction moment
    double helpvar_07 = friction_coefficient_rolling_mat[type_i][type_j]*rad_i*fce_coll_norm_ij_mag;
    double mom_fric_x_ij = -unit_relangvel_x_ij*helpvar_07;
    double mom_fric_y_ij = -unit_relangvel_y_ij*helpvar_07;
    double mom_fric_z_ij = -unit_relangvel_z_ij*helpvar_07;

    // update collision matrix
    smat_set_value(relative_velocity_tangent_smat, id_i, id_j, relvel_tang_ij_mag);
    
    // add forces and moments
    sphere_fms.force_sum_x_vec[indx_i] += fce_coll_x_ij;
    sphere_fms.force_sum_y_vec[indx_i] += fce_coll_y_ij;
    sphere_fms.force_sum_z_vec[indx_i] += fce_coll_z_ij;
    sphere_fms.moment_sum_x_vec[indx_i] += mom_coll_x_ij + mom_fric_x_ij;
    sphere_fms.moment_sum_y_vec[indx_i] += mom_coll_y_ij + mom_fric_y_ij;
    sphere_fms.moment_sum_z_vec[indx_i] += mom_coll_z_ij + mom_fric_z_ij;

    // apply Newton's third law to get forces on j
    // collision force and friction moment are negated
    // collion moment remains the same
    sphere_fms.force_sum_x_vec[indx_j] += -fce_coll_x_ij;
    sphere_fms.force_sum_y_vec[indx_j] += -fce_coll_y_ij;
    sphere_fms.force_sum_z_vec[indx_j] += -fce_coll_z_ij;
    sphere_fms.moment_sum_x_vec[indx_j] += mom_coll_x_ij - mom_fric_x_ij;
    sphere_fms.moment_sum_y_vec[indx_j] += mom_coll_y_ij - mom_fric_y_ij;
    sphere_fms.moment_sum_z_vec[indx_j] += mom_coll_z_ij - mom_fric_z_ij;

}

#endif
