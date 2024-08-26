#ifndef OUTPUT_FORCE_FORCE_SPHEREWALLMESH_HERTZ
#define OUTPUT_FORCE_FORCE_SPHEREWALLMESH_HERTZ
#include <fstream>
#include <vector>
#include "collisioncheck_spherewallmesh_naive.hpp"
#include "container_function.hpp"
#include "container_sphere.hpp"
#include "container_typedef.hpp"
#include "container_wallmesh.hpp"


class OutputForceForceSphereWallMeshHertz
{

    public:

    // constant
    double TOLERANCE = 1e-10;

    // variables
    VectorDouble radius_vec;
    MatrixDouble spring_constant_normal_mat;
    MatrixDouble spring_constant_tangent_mat;
    MatrixDouble damping_coefficient_normal_mat;
    MatrixDouble damping_coefficient_tangent_mat;
    MatrixDouble friction_coefficient_sliding_mat;
    MatrixDouble friction_coefficient_rolling_mat;
    std::string file_out_base_str;
    std::ofstream file_out_stream;
    double pos_reference_x;
    double pos_reference_y;
    double pos_reference_z;

    // collision checker
    CollisionCheckSphereWallMeshNaive collision_check;

    // functions
    void add_force_moment(
        SphereForceMomentStruct &sphere_fms,
        SparseMatrixDouble &overlap_tangent_smat,
        SparseMatrixDouble &relative_velocity_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int ts
    );

    // constructor
    OutputForceForceSphereWallMeshHertz(
        VectorDouble radius_vec_in,
        MatrixDouble spring_constant_normal_mat_in,
        MatrixDouble spring_constant_tangent_mat_in,
        MatrixDouble damping_coefficient_normal_mat_in,
        MatrixDouble damping_coefficient_tangent_mat_in,
        MatrixDouble friction_coefficient_sliding_mat_in,
        MatrixDouble friction_coefficient_rolling_mat_in,
        std::string file_out_base_str_in,
        double pos_reference_x_in,
        double pos_reference_y_in,
        double pos_reference_z_in
    )
    {
        
        // store variables
        radius_vec = radius_vec_in;
        spring_constant_normal_mat = spring_constant_normal_mat_in;
        spring_constant_tangent_mat = spring_constant_tangent_mat_in;
        damping_coefficient_normal_mat = damping_coefficient_normal_mat_in;
        damping_coefficient_tangent_mat = damping_coefficient_tangent_mat_in;
        friction_coefficient_sliding_mat = friction_coefficient_sliding_mat_in;
        friction_coefficient_rolling_mat = friction_coefficient_rolling_mat_in;
        file_out_base_str = file_out_base_str_in;
        pos_reference_x = pos_reference_x_in;
        pos_reference_y = pos_reference_y_in;
        pos_reference_z = pos_reference_z_in;

        // initialize output file
        std::string file_out_str = file_out_base_str + ".csv";
        file_out_stream.open(file_out_str);
        file_out_stream << "ts,fce_x_ki,fce_y_ki,fce_z_ki,mom_x_ki,mom_y_ki,mom_z_ki\n";

    }

    private:
    void calculate_force_moment(
        SphereForceMomentStruct &wallmesh_fms,
        SphereForceMomentStruct &sphere_fms,
        SparseMatrixDouble &overlap_tangent_smat,
        SparseMatrixDouble &relative_velocity_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int indx_i, int indx_k
    );
    bool check_possible_collision(
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int indx_i, int indx_k
    );
    void check_face_collision(
        bool &is_face_collision, 
        double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int indx_i, int indx_k
    );
    void check_edge_collision(
        bool &is_edge_collision,
        double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int indx_i, int indx_k
    );
    void check_vertex_collision(
        bool &is_vertex_collision,
        double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int indx_i, int indx_k
    );
    void calculate_velocity_contact(
        double &vel_contact_x, double &vel_contact_y, double &vel_contact_z,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        double pos_contact_x, double pos_contact_y, double pos_contact_z
    );

};

void OutputForceForceSphereWallMeshHertz::add_force_moment(
    SphereForceMomentStruct &sphere_fms,
    SparseMatrixDouble &overlap_tangent_smat,
    SparseMatrixDouble &relative_velocity_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int ts
)
{

    // initialize forces and moments
    SphereForceMomentStruct wallmesh_fms = sphere_fms_fill(sphere_fms.num_particle);

    // generate preliminary list of collision pairs
    VectorPairInt collision_vec = collision_check.broad_search(sphere_pvs, wallmesh_pvs);

    // calculate force for each collision pair
    // compute moment on wallmesh due to lever arm
    for (auto &collision_pair : collision_vec)
    {
        
        // get indices of particles
        int indx_i = collision_pair.first;
        int indx_k = collision_pair.second;

        // calculate forces
        calculate_force_moment(wallmesh_fms, sphere_fms, overlap_tangent_smat, relative_velocity_tangent_smat, sphere_pvs, wallmesh_pvs, indx_i, indx_k);
        
    }

    // initialize forces on wallmesh
    double frc_x_ki = 0.; 
    double frc_y_ki = 0.; 
    double frc_z_ki = 0.; 
    double mom_x_ki = 0.;
    double mom_y_ki = 0.;
    double mom_z_ki = 0.;

    // update positions and velocities
    for (int indx_i = 0; indx_i < wallmesh_fms.num_particle; indx_i++)
    {
        
        // calculate number of contacts for averaging
        int num_contact = wallmesh_fms.num_contact_vec[indx_i];
        double inv_num_contact = 0.;
        if (num_contact != 0)
        {
            inv_num_contact = 1./(double) num_contact;
        }

        // calculate net forces and moments
        frc_x_ki += wallmesh_fms.force_sum_x_vec[indx_i] + inv_num_contact*wallmesh_fms.force_average_x_vec[indx_i];
        frc_y_ki += wallmesh_fms.force_sum_y_vec[indx_i] + inv_num_contact*wallmesh_fms.force_average_y_vec[indx_i];
        frc_z_ki += wallmesh_fms.force_sum_z_vec[indx_i] + inv_num_contact*wallmesh_fms.force_average_z_vec[indx_i];
        mom_x_ki += wallmesh_fms.moment_sum_x_vec[indx_i] + inv_num_contact*wallmesh_fms.moment_average_x_vec[indx_i];
        mom_y_ki += wallmesh_fms.moment_sum_y_vec[indx_i] + inv_num_contact*wallmesh_fms.moment_average_y_vec[indx_i];
        mom_z_ki += wallmesh_fms.moment_sum_z_vec[indx_i] + inv_num_contact*wallmesh_fms.moment_average_z_vec[indx_i];

    }

    // write to file
    file_out_stream << ts << ",";
    file_out_stream << frc_x_ki << ",";
    file_out_stream << frc_y_ki << ",";
    file_out_stream << frc_z_ki << ",";
    file_out_stream << mom_x_ki << ",";
    file_out_stream << mom_y_ki << ",";
    file_out_stream << mom_z_ki << "\n";

}

void OutputForceForceSphereWallMeshHertz::calculate_force_moment(
    SphereForceMomentStruct &wallmesh_fms,
    SphereForceMomentStruct &sphere_fms,
    SparseMatrixDouble &overlap_tangent_smat,
    SparseMatrixDouble &relative_velocity_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{

    // get particle and wall id
    int id_i = sphere_pvs.id_vec[indx_i];
    int id_k = wallmesh_pvs.id_vec[indx_k];

    // skip if too far for collision
    bool is_possible_collision = check_possible_collision(sphere_pvs, wallmesh_pvs, indx_i, indx_k);
    if (!is_possible_collision)
    {
        smat_prune(overlap_tangent_smat, id_i, id_k);  // reset tangential overlap
        smat_prune(relative_velocity_tangent_smat, id_i, id_k);
        return;
    }

    // initialize contact point
    double cont_x_ik = 0.;
    double cont_y_ik = 0.;
    double cont_z_ik = 0.;

    // initialize collision type indicator
    bool is_face_collision = false;
    bool is_edge_collision = false;
    bool is_vertex_collision = false;

    // check if face collision
    check_face_collision(is_face_collision, cont_x_ik, cont_y_ik, cont_z_ik, sphere_pvs, wallmesh_pvs, indx_i, indx_k);
    if (!is_face_collision)
    {

        // check if edge collision
        check_edge_collision(is_edge_collision, cont_x_ik, cont_y_ik, cont_z_ik, sphere_pvs, wallmesh_pvs, indx_i, indx_k);
        if (!is_edge_collision)
        {
            
            // check if vertex collision
            check_vertex_collision(is_vertex_collision, cont_x_ik, cont_y_ik, cont_z_ik, sphere_pvs, wallmesh_pvs, indx_i, indx_k);
            
            // skip if no collision
            if (!is_vertex_collision)
            {
                smat_prune(overlap_tangent_smat, id_i, id_k);  // reset tangential overlap
                smat_prune(relative_velocity_tangent_smat, id_i, id_k);
                return;
            }

        }

    }

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

    // get particle radius
    double rad_i = radius_vec[type_i];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];  

    // get wall points
    double pos_p1_x_k = wallmesh_pvs.position_p1_x_vec[indx_k];
    double pos_p1_y_k = wallmesh_pvs.position_p1_y_vec[indx_k];
    double pos_p1_z_k = wallmesh_pvs.position_p1_z_vec[indx_k];
    double pos_p2_x_k = wallmesh_pvs.position_p2_x_vec[indx_k];
    double pos_p2_y_k = wallmesh_pvs.position_p2_y_vec[indx_k];
    double pos_p2_z_k = wallmesh_pvs.position_p2_z_vec[indx_k];
    double pos_p3_x_k = wallmesh_pvs.position_p3_x_vec[indx_k];
    double pos_p3_y_k = wallmesh_pvs.position_p3_y_vec[indx_k];
    double pos_p3_z_k = wallmesh_pvs.position_p3_z_vec[indx_k];

    // calculate normal vector
    double helpvar_01 = 1./sqrt((cont_x_ik - pos_x_i)*(cont_x_ik - pos_x_i) + (cont_y_ik - pos_y_i)*(cont_y_ik - pos_y_i) + (cont_z_ik - pos_z_i)*(cont_z_ik - pos_z_i));
    double norm_x_ik = helpvar_01*(cont_x_ik - pos_x_i);
    double norm_y_ik = helpvar_01*(cont_y_ik - pos_y_i);
    double norm_z_ik = helpvar_01*(cont_z_ik - pos_z_i);

    // calculate velocity at contact point
    double vel_x_k = 0;
    double vel_y_k = 0;
    double vel_z_k = 0;
    calculate_velocity_contact(vel_x_k, vel_y_k, vel_z_k, wallmesh_pvs, cont_x_ik, cont_y_ik, cont_z_ik);

    // get particle velocities
    double vel_x_i = sphere_pvs.velocity_x_vec[indx_i];
    double vel_y_i = sphere_pvs.velocity_y_vec[indx_i];
    double vel_z_i = sphere_pvs.velocity_z_vec[indx_i];

    // get particle angular velocities
    double angvel_x_i = sphere_pvs.angularvelocity_x_vec[indx_i];
    double angvel_y_i = sphere_pvs.angularvelocity_y_vec[indx_i];
    double angvel_z_i = sphere_pvs.angularvelocity_z_vec[indx_i];

    // get tangential overlap
    double overlap_tang_ik_mag = smat_get_value(overlap_tangent_smat, id_i, id_k);

    // calculate relative velocity vector
    double relvel_x_ik = rad_i*(angvel_y_i*norm_z_ik - angvel_z_i*norm_y_ik) + vel_x_i - vel_x_k;
    double relvel_y_ik = -rad_i*(angvel_x_i*norm_z_ik - angvel_z_i*norm_x_ik) + vel_y_i - vel_y_k;
    double relvel_z_ik = rad_i*(angvel_x_i*norm_y_ik - angvel_y_i*norm_x_ik) + vel_z_i - vel_z_k;

    // calculate normal component of relative velocity vector
    double helpvar_02 = (norm_x_ik*relvel_x_ik + norm_y_ik*relvel_y_ik + norm_z_ik*relvel_z_ik);
    double relvel_norm_x_ik = norm_x_ik*helpvar_02;
    double relvel_norm_y_ik = norm_y_ik*helpvar_02;
    double relvel_norm_z_ik = norm_z_ik*helpvar_02;

    // calculate tangential component of relative velocity vector
    double relvel_tang_x_ik = relvel_x_ik - relvel_norm_x_ik;
    double relvel_tang_y_ik = relvel_y_ik - relvel_norm_y_ik;
    double relvel_tang_z_ik = relvel_z_ik - relvel_norm_z_ik;

    // calculate tangential vector
    // tangential vector is zero if relative velocity (tangential component) is zero
    double helpvar_03 = 1./sqrt(relvel_tang_x_ik*relvel_tang_x_ik + relvel_tang_y_ik*relvel_tang_y_ik + relvel_tang_z_ik*relvel_tang_z_ik);
    double tang_x_ik = relvel_tang_x_ik*helpvar_03;
    double tang_y_ik = relvel_tang_y_ik*helpvar_03;
    double tang_z_ik = relvel_tang_z_ik*helpvar_03;
    if (abs(relvel_tang_x_ik) < TOLERANCE && abs(relvel_tang_y_ik) < TOLERANCE && abs(relvel_tang_z_ik) < TOLERANCE)
    {
        tang_x_ik = 0.;
        tang_y_ik = 0.;
        tang_z_ik = 0.;
    }

    // calculate normal component of relative velocity
    double relvel_norm_ik_mag = norm_x_ik*relvel_norm_x_ik + norm_y_ik*relvel_norm_y_ik + norm_z_ik*relvel_norm_z_ik;

    // calculate normal overlap
    double overlap_norm_ik_mag = rad_i - sqrt((cont_x_ik - pos_x_i)*(cont_x_ik - pos_x_i) + (cont_y_ik - pos_y_i)*(cont_y_ik - pos_y_i) + (cont_z_ik - pos_z_i)*(cont_z_ik - pos_z_i));

    // calculate normal component of collision force
    double helpvar_04 = overlap_norm_ik_mag*spring_constant_normal_mat[type_i][type_k] + relvel_norm_ik_mag*damping_coefficient_normal_mat[type_i][type_k];
    double fce_coll_norm_x_ik = -norm_x_ik*helpvar_04;
    double fce_coll_norm_y_ik = -norm_y_ik*helpvar_04;
    double fce_coll_norm_z_ik = -norm_z_ik*helpvar_04;

    // calculate tangential component of relative velocity
    double relvel_tang_ik_mag = -tang_x_ik*(norm_y_ik*angvel_z_i*rad_i - norm_z_ik*angvel_y_i*rad_i - vel_x_i + vel_x_k) + tang_y_ik*(norm_x_ik*angvel_z_i*rad_i - norm_z_ik*angvel_x_i*rad_i + vel_y_i - vel_y_k) - tang_z_ik*(norm_x_ik*angvel_y_i*rad_i - norm_y_ik*angvel_x_i*rad_i - vel_z_i + vel_z_k);

    // calculate tangential component of collision force (hertzian contact)
    double helpvar_05 = overlap_tang_ik_mag*spring_constant_tangent_mat[type_i][type_k] + relvel_tang_ik_mag*damping_coefficient_tangent_mat[type_i][type_k];
    double fce_coll_tang_x_ik = -tang_x_ik*helpvar_05;
    double fce_coll_tang_y_ik = -tang_y_ik*helpvar_05;
    double fce_coll_tang_z_ik = -tang_z_ik*helpvar_05;

    // calculate magnitude of normal and tangential collision forces
    double fce_coll_norm_ik_mag = sqrt(fce_coll_norm_x_ik*fce_coll_norm_x_ik + fce_coll_norm_y_ik*fce_coll_norm_y_ik + fce_coll_norm_z_ik*fce_coll_norm_z_ik);
    double fce_coll_tang_ik_mag = sqrt(fce_coll_tang_x_ik*fce_coll_tang_x_ik + fce_coll_tang_y_ik*fce_coll_tang_y_ik + fce_coll_tang_z_ik*fce_coll_tang_z_ik);

    // recalculate tangential force if it exceeds maximum static friction
    if (fce_coll_tang_ik_mag > friction_coefficient_sliding_mat[type_i][type_k]*fce_coll_norm_ik_mag)
    {
        
        // calcualte signum of tangential overlap
        double sgn_overlap_tang_ik_mag = 1.;
        if (overlap_tang_ik_mag < 0.)
        {
            sgn_overlap_tang_ik_mag = -1.;
        }
        
        // calculate tangential component of collision force (friction)
        double helpvar_06 = friction_coefficient_sliding_mat[type_i][type_k]*sgn_overlap_tang_ik_mag*fce_coll_norm_ik_mag;
        fce_coll_tang_x_ik = -tang_x_ik*helpvar_06;
        fce_coll_tang_y_ik = -tang_y_ik*helpvar_06;
        fce_coll_tang_z_ik = -tang_z_ik*helpvar_06;

    }

    // calculate collision force
    double fce_coll_x_ik = fce_coll_norm_x_ik + fce_coll_tang_x_ik;
    double fce_coll_y_ik = fce_coll_norm_y_ik + fce_coll_tang_y_ik;
    double fce_coll_z_ik = fce_coll_norm_z_ik + fce_coll_tang_z_ik;

    // calculate collision moment
    double mom_coll_x_ik = rad_i*(-fce_coll_y_ik*norm_z_ik + fce_coll_z_ik*norm_y_ik);
    double mom_coll_y_ik = rad_i*(fce_coll_x_ik*norm_z_ik - fce_coll_z_ik*norm_x_ik);
    double mom_coll_z_ik = rad_i*(-fce_coll_x_ik*norm_y_ik + fce_coll_y_ik*norm_x_ik);

    // calculate unit vector pointing to relative angular velocity
    // unit vector is zero if relative angular velocity is zero
    double helpvar_07 = 1./sqrt(angvel_x_i*angvel_x_i + angvel_y_i*angvel_y_i + angvel_z_i*angvel_z_i);
    double unit_relangvel_x_ik = angvel_x_i*helpvar_07;
    double unit_relangvel_y_ik = angvel_y_i*helpvar_07;
    double unit_relangvel_z_ik = angvel_z_i*helpvar_07;
    if (abs(angvel_x_i) < TOLERANCE && abs(angvel_y_i) < TOLERANCE && abs(angvel_z_i) < TOLERANCE)
    {
        unit_relangvel_x_ik = 0.;
        unit_relangvel_y_ik = 0.;
        unit_relangvel_z_ik = 0.;
    }

    // calculate friction moment
    double helpvar_08 = friction_coefficient_rolling_mat[type_i][type_k]*rad_i*fce_coll_norm_ik_mag;
    double mom_fric_x_ik = -unit_relangvel_x_ik*helpvar_08;
    double mom_fric_y_ik = -unit_relangvel_y_ik*helpvar_08;
    double mom_fric_z_ik = -unit_relangvel_z_ik*helpvar_08;

    // update collision matrix
    smat_set_value(relative_velocity_tangent_smat, id_i, id_k, relvel_tang_ik_mag);

    // get lever arm from centroid to particle
    double dpos_ref_cont_x = cont_x_ik - pos_reference_x;
    double dpos_ref_cont_y = cont_y_ik - pos_reference_y;
    double dpos_ref_cont_z = cont_z_ik - pos_reference_z;

    // face collision -> add to total force
    // edge or vertex collision -> add to average force
    if (is_face_collision)
    {
        
        // forces on spheres
        sphere_fms.force_sum_x_vec[indx_i] += fce_coll_x_ik;
        sphere_fms.force_sum_y_vec[indx_i] += fce_coll_y_ik;
        sphere_fms.force_sum_z_vec[indx_i] += fce_coll_z_ik;
        sphere_fms.moment_sum_x_vec[indx_i] += mom_coll_x_ik + mom_fric_x_ik;
        sphere_fms.moment_sum_y_vec[indx_i] += mom_coll_y_ik + mom_fric_y_ik;
        sphere_fms.moment_sum_z_vec[indx_i] += mom_coll_z_ik + mom_fric_z_ik;

        // forces on wallmesh
        wallmesh_fms.force_sum_x_vec[indx_i] -= fce_coll_x_ik;
        wallmesh_fms.force_sum_y_vec[indx_i] -= fce_coll_y_ik;
        wallmesh_fms.force_sum_z_vec[indx_i] -= fce_coll_z_ik;
        wallmesh_fms.moment_sum_x_vec[indx_i] -= +dpos_ref_cont_y*fce_coll_z_ik - dpos_ref_cont_z*fce_coll_y_ik;
        wallmesh_fms.moment_sum_y_vec[indx_i] -= -dpos_ref_cont_x*fce_coll_z_ik + dpos_ref_cont_z*fce_coll_x_ik;
        wallmesh_fms.moment_sum_z_vec[indx_i] -= +dpos_ref_cont_x*fce_coll_y_ik - dpos_ref_cont_y*fce_coll_x_ik;

    }
    else if (is_edge_collision || is_vertex_collision)
    {
        
        // forces on spheres
        sphere_fms.force_average_x_vec[indx_i] += fce_coll_x_ik;
        sphere_fms.force_average_y_vec[indx_i] += fce_coll_y_ik;
        sphere_fms.force_average_z_vec[indx_i] += fce_coll_z_ik;
        sphere_fms.moment_average_x_vec[indx_i] += mom_coll_x_ik + mom_fric_x_ik;
        sphere_fms.moment_average_y_vec[indx_i] += mom_coll_y_ik + mom_fric_y_ik;
        sphere_fms.moment_average_z_vec[indx_i] += mom_coll_z_ik + mom_fric_z_ik;
        sphere_fms.num_contact_vec[indx_i]++;  // increment contact count

        // forces on wallmesh
        wallmesh_fms.force_average_x_vec[indx_i] -= fce_coll_x_ik;
        wallmesh_fms.force_average_y_vec[indx_i] -= fce_coll_y_ik;
        wallmesh_fms.force_average_z_vec[indx_i] -= fce_coll_z_ik;
        wallmesh_fms.moment_sum_x_vec[indx_i] -= +dpos_ref_cont_y*fce_coll_z_ik - dpos_ref_cont_z*fce_coll_y_ik;
        wallmesh_fms.moment_sum_y_vec[indx_i] -= -dpos_ref_cont_x*fce_coll_z_ik + dpos_ref_cont_z*fce_coll_x_ik;
        wallmesh_fms.moment_sum_z_vec[indx_i] -= +dpos_ref_cont_x*fce_coll_y_ik - dpos_ref_cont_y*fce_coll_x_ik;
        wallmesh_fms.num_contact_vec[indx_i]++;  // increment contact count

    }

}

bool OutputForceForceSphereWallMeshHertz::check_possible_collision(
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

    // get particle radius
    double rad_i = radius_vec[type_i];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];  

    // get wall points
    double pos_p1_x_k = wallmesh_pvs.position_p1_x_vec[indx_k];
    double pos_p1_y_k = wallmesh_pvs.position_p1_y_vec[indx_k];
    double pos_p1_z_k = wallmesh_pvs.position_p1_z_vec[indx_k];
    double pos_p2_x_k = wallmesh_pvs.position_p2_x_vec[indx_k];
    double pos_p2_y_k = wallmesh_pvs.position_p2_y_vec[indx_k];
    double pos_p2_z_k = wallmesh_pvs.position_p2_z_vec[indx_k];
    double pos_p3_x_k = wallmesh_pvs.position_p3_x_vec[indx_k];
    double pos_p3_y_k = wallmesh_pvs.position_p3_y_vec[indx_k];
    double pos_p3_z_k = wallmesh_pvs.position_p3_z_vec[indx_k];

    // calculate vector facing the "front" of the triangle
    double frnt_x_ki = (pos_p1_y_k - pos_p2_y_k)*(pos_p1_z_k - pos_p3_z_k) - (pos_p1_y_k - pos_p3_y_k)*(pos_p1_z_k - pos_p2_z_k);
    double frnt_y_ki = -(pos_p1_x_k - pos_p2_x_k)*(pos_p1_z_k - pos_p3_z_k) + (pos_p1_x_k - pos_p3_x_k)*(pos_p1_z_k - pos_p2_z_k);
    double frnt_z_ki = (pos_p1_x_k - pos_p2_x_k)*(pos_p1_y_k - pos_p3_y_k) - (pos_p1_x_k - pos_p3_x_k)*(pos_p1_y_k - pos_p2_y_k);

    // calculate front facing normal vector
    double helpvar_01 = 1./sqrt(frnt_x_ki*frnt_x_ki + frnt_y_ki*frnt_y_ki + frnt_z_ki*frnt_z_ki);
    double norm_frnt_x_ki = frnt_x_ki*helpvar_01;
    double norm_frnt_y_ki = frnt_y_ki*helpvar_01;
    double norm_frnt_z_ki = frnt_z_ki*helpvar_01;

    // calculate back facing normal vector
    double norm_back_x_ki = -norm_frnt_x_ki;
    double norm_back_y_ki = -norm_frnt_y_ki;
    double norm_back_z_ki = -norm_frnt_z_ki;

    // calculate front facing wall to particle distance
    double dist_frnt_ki_mag = -norm_frnt_x_ki*(pos_p1_x_k - pos_x_i) - norm_frnt_y_ki*(pos_p1_y_k - pos_y_i) - norm_frnt_z_ki*(pos_p1_z_k - pos_z_i);

    // calculate back facing wall to particle distance
    double dist_back_ki_mag = -dist_frnt_ki_mag;

    // determine whether to use front or back facing normal and distance
    // particle faces the front if the distance from the front is positive
    double dist_ki_mag = 0.;
    double norm_x_ki = 0.;
    double norm_y_ki = 0.;
    double norm_z_ki = 0.;
    if (dist_frnt_ki_mag >= 0)
    {
        dist_ki_mag = dist_frnt_ki_mag;
        norm_x_ki = norm_frnt_x_ki;
        norm_y_ki = norm_frnt_y_ki;
        norm_z_ki = norm_frnt_z_ki;
    }
    else
    {
        dist_ki_mag = dist_back_ki_mag;
        norm_x_ki = norm_back_x_ki;
        norm_y_ki = norm_back_y_ki;
        norm_z_ki = norm_back_z_ki;
    }

    // false if particle is too far for collision
    return dist_ki_mag < rad_i;

}

void OutputForceForceSphereWallMeshHertz::check_face_collision(
    bool &is_face_collision, 
    double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

    // get particle radius
    double rad_i = radius_vec[type_i];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];  

    // get wall points
    double pos_p1_x_k = wallmesh_pvs.position_p1_x_vec[indx_k];
    double pos_p1_y_k = wallmesh_pvs.position_p1_y_vec[indx_k];
    double pos_p1_z_k = wallmesh_pvs.position_p1_z_vec[indx_k];
    double pos_p2_x_k = wallmesh_pvs.position_p2_x_vec[indx_k];
    double pos_p2_y_k = wallmesh_pvs.position_p2_y_vec[indx_k];
    double pos_p2_z_k = wallmesh_pvs.position_p2_z_vec[indx_k];
    double pos_p3_x_k = wallmesh_pvs.position_p3_x_vec[indx_k];
    double pos_p3_y_k = wallmesh_pvs.position_p3_y_vec[indx_k];
    double pos_p3_z_k = wallmesh_pvs.position_p3_z_vec[indx_k];

    // calculate vector facing the "front" of the triangle
    double frnt_x_ki = (pos_p1_y_k - pos_p2_y_k)*(pos_p1_z_k - pos_p3_z_k) - (pos_p1_y_k - pos_p3_y_k)*(pos_p1_z_k - pos_p2_z_k);
    double frnt_y_ki = -(pos_p1_x_k - pos_p2_x_k)*(pos_p1_z_k - pos_p3_z_k) + (pos_p1_x_k - pos_p3_x_k)*(pos_p1_z_k - pos_p2_z_k);
    double frnt_z_ki = (pos_p1_x_k - pos_p2_x_k)*(pos_p1_y_k - pos_p3_y_k) - (pos_p1_x_k - pos_p3_x_k)*(pos_p1_y_k - pos_p2_y_k);

    // calculate front facing normal vector
    double helpvar_01 = 1./sqrt(frnt_x_ki*frnt_x_ki + frnt_y_ki*frnt_y_ki + frnt_z_ki*frnt_z_ki);
    double norm_frnt_x_ki = frnt_x_ki*helpvar_01;
    double norm_frnt_y_ki = frnt_y_ki*helpvar_01;
    double norm_frnt_z_ki = frnt_z_ki*helpvar_01;

    // calculate back facing normal vector
    double norm_back_x_ki = -norm_frnt_x_ki;
    double norm_back_y_ki = -norm_frnt_y_ki;
    double norm_back_z_ki = -norm_frnt_z_ki;

    // calculate front facing wall to particle distance
    double dist_frnt_ki_mag = -norm_frnt_x_ki*(pos_p1_x_k - pos_x_i) - norm_frnt_y_ki*(pos_p1_y_k - pos_y_i) - norm_frnt_z_ki*(pos_p1_z_k - pos_z_i);

    // calculate back facing wall to particle distance
    double dist_back_ki_mag = -dist_frnt_ki_mag;

    // determine whether to use front or back facing normal and distance
    // particle faces the front if the distance from the front is positive
    double dist_ki_mag = 0.;
    double norm_x_ki = 0.;
    double norm_y_ki = 0.;
    double norm_z_ki = 0.;
    if (dist_frnt_ki_mag >= 0)
    {
        dist_ki_mag = dist_frnt_ki_mag;
        norm_x_ki = norm_frnt_x_ki;
        norm_y_ki = norm_frnt_y_ki;
        norm_z_ki = norm_frnt_z_ki;
    }
    else
    {
        dist_ki_mag = dist_back_ki_mag;
        norm_x_ki = norm_back_x_ki;
        norm_y_ki = norm_back_y_ki;
        norm_z_ki = norm_back_z_ki;
    }

    // calculate preliminary contact point
    double prelcont_x_ki = -dist_ki_mag*norm_x_ki + pos_x_i;
    double prelcont_y_ki = -dist_ki_mag*norm_y_ki + pos_y_i;
    double prelcont_z_ki = -dist_ki_mag*norm_z_ki + pos_z_i;

    // calculate test vectors
    double facetestvec1_x_ki = (pos_p1_y_k - prelcont_y_ki)*(pos_p2_z_k - prelcont_z_ki) - (pos_p1_z_k - prelcont_z_ki)*(pos_p2_y_k - prelcont_y_ki);
    double facetestvec1_y_ki = -(pos_p1_x_k - prelcont_x_ki)*(pos_p2_z_k - prelcont_z_ki) + (pos_p1_z_k - prelcont_z_ki)*(pos_p2_x_k - prelcont_x_ki);
    double facetestvec1_z_ki = (pos_p1_x_k - prelcont_x_ki)*(pos_p2_y_k - prelcont_y_ki) - (pos_p1_y_k - prelcont_y_ki)*(pos_p2_x_k - prelcont_x_ki);
    double facetestvec2_x_ki = (pos_p2_y_k - prelcont_y_ki)*(pos_p3_z_k - prelcont_z_ki) - (pos_p2_z_k - prelcont_z_ki)*(pos_p3_y_k - prelcont_y_ki);
    double facetestvec2_y_ki = -(pos_p2_x_k - prelcont_x_ki)*(pos_p3_z_k - prelcont_z_ki) + (pos_p2_z_k - prelcont_z_ki)*(pos_p3_x_k - prelcont_x_ki);
    double facetestvec2_z_ki = (pos_p2_x_k - prelcont_x_ki)*(pos_p3_y_k - prelcont_y_ki) - (pos_p2_y_k - prelcont_y_ki)*(pos_p3_x_k - prelcont_x_ki);
    double facetestvec3_x_ki = -(pos_p1_y_k - prelcont_y_ki)*(pos_p3_z_k - prelcont_z_ki) + (pos_p1_z_k - prelcont_z_ki)*(pos_p3_y_k - prelcont_y_ki);
    double facetestvec3_y_ki = (pos_p1_x_k - prelcont_x_ki)*(pos_p3_z_k - prelcont_z_ki) - (pos_p1_z_k - prelcont_z_ki)*(pos_p3_x_k - prelcont_x_ki);
    double facetestvec3_z_ki = -(pos_p1_x_k - prelcont_x_ki)*(pos_p3_y_k - prelcont_y_ki) + (pos_p1_y_k - prelcont_y_ki)*(pos_p3_x_k - prelcont_x_ki);
    double facetestcrit1_ki_mag = facetestvec1_x_ki*facetestvec2_x_ki + facetestvec1_y_ki*facetestvec2_y_ki + facetestvec1_z_ki*facetestvec2_z_ki;
    double facetestcrit2_ki_mag = facetestvec1_x_ki*facetestvec3_x_ki + facetestvec1_y_ki*facetestvec3_y_ki + facetestvec1_z_ki*facetestvec3_z_ki;

    // calculate criteria for face collision
    is_face_collision = facetestcrit1_ki_mag > TOLERANCE && facetestcrit2_ki_mag > TOLERANCE;

    // record contact point
    if (is_face_collision)
    {
        pos_contact_x = prelcont_x_ki;
        pos_contact_y = prelcont_y_ki;
        pos_contact_z = prelcont_z_ki;
    }

}

void OutputForceForceSphereWallMeshHertz::check_edge_collision(
    bool &is_edge_collision,
    double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

    // get particle radius
    double rad_i = radius_vec[type_i];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];  

    // get wall points
    double pos_p1_x_k = wallmesh_pvs.position_p1_x_vec[indx_k];
    double pos_p1_y_k = wallmesh_pvs.position_p1_y_vec[indx_k];
    double pos_p1_z_k = wallmesh_pvs.position_p1_z_vec[indx_k];
    double pos_p2_x_k = wallmesh_pvs.position_p2_x_vec[indx_k];
    double pos_p2_y_k = wallmesh_pvs.position_p2_y_vec[indx_k];
    double pos_p2_z_k = wallmesh_pvs.position_p2_z_vec[indx_k];
    double pos_p3_x_k = wallmesh_pvs.position_p3_x_vec[indx_k];
    double pos_p3_y_k = wallmesh_pvs.position_p3_y_vec[indx_k];
    double pos_p3_z_k = wallmesh_pvs.position_p3_z_vec[indx_k];

    // calculate distance from edge P1 to P2 and center
    double dpos_p1p2_x_k = -pos_p1_x_k + pos_p2_x_k;
    double dpos_p1p2_y_k = -pos_p1_y_k + pos_p2_y_k;
    double dpos_p1p2_z_k = -pos_p1_z_k + pos_p2_z_k;
    double relproj_p1p2_ki_mag = (-dpos_p1p2_x_k*(pos_p1_x_k - pos_x_i) - dpos_p1p2_y_k*(pos_p1_y_k - pos_y_i) - dpos_p1p2_z_k*(pos_p1_z_k - pos_z_i))/(dpos_p1p2_x_k*dpos_p1p2_x_k + dpos_p1p2_y_k*dpos_p1p2_y_k + dpos_p1p2_z_k*dpos_p1p2_z_k);
    double proj_p1p2_x_ki = dpos_p1p2_x_k*relproj_p1p2_ki_mag + pos_p1_x_k;
    double proj_p1p2_y_ki = dpos_p1p2_y_k*relproj_p1p2_ki_mag + pos_p1_y_k;
    double proj_p1p2_z_ki = dpos_p1p2_z_k*relproj_p1p2_ki_mag + pos_p1_z_k;
    double dist_p1p2_ki_mag = sqrt((pos_x_i - proj_p1p2_x_ki)*(pos_x_i - proj_p1p2_x_ki) + (pos_y_i - proj_p1p2_y_ki)*(pos_y_i - proj_p1p2_y_ki) + (pos_z_i - proj_p1p2_z_ki)*(pos_z_i - proj_p1p2_z_ki));

    // calculate distance from edge P2 to P3 and center
    double dpos_p2p3_x_k = -pos_p2_x_k + pos_p3_x_k;
    double dpos_p2p3_y_k = -pos_p2_y_k + pos_p3_y_k;
    double dpos_p2p3_z_k = -pos_p2_z_k + pos_p3_z_k;
    double relproj_p2p3_ki_mag = (-dpos_p2p3_x_k*(pos_p2_x_k - pos_x_i) - dpos_p2p3_y_k*(pos_p2_y_k - pos_y_i) - dpos_p2p3_z_k*(pos_p2_z_k - pos_z_i))/(dpos_p2p3_x_k*dpos_p2p3_x_k + dpos_p2p3_y_k*dpos_p2p3_y_k + dpos_p2p3_z_k*dpos_p2p3_z_k);
    double proj_p2p3_x_ki = dpos_p2p3_x_k*relproj_p2p3_ki_mag + pos_p2_x_k;
    double proj_p2p3_y_ki = dpos_p2p3_y_k*relproj_p2p3_ki_mag + pos_p2_y_k;
    double proj_p2p3_z_ki = dpos_p2p3_z_k*relproj_p2p3_ki_mag + pos_p2_z_k;
    double dist_p2p3_ki_mag = sqrt((pos_x_i - proj_p2p3_x_ki)*(pos_x_i - proj_p2p3_x_ki) + (pos_y_i - proj_p2p3_y_ki)*(pos_y_i - proj_p2p3_y_ki) + (pos_z_i - proj_p2p3_z_ki)*(pos_z_i - proj_p2p3_z_ki));

    // calculate distance from edge P3 to P1 and center
    double dpos_p3p1_x_k = -pos_p3_x_k + pos_p1_x_k;
    double dpos_p3p1_y_k = -pos_p3_y_k + pos_p1_y_k;
    double dpos_p3p1_z_k = -pos_p3_z_k + pos_p1_z_k;
    double relproj_p3p1_ki_mag = (-dpos_p3p1_x_k*(pos_p3_x_k - pos_x_i) - dpos_p3p1_y_k*(pos_p3_y_k - pos_y_i) - dpos_p3p1_z_k*(pos_p3_z_k - pos_z_i))/(dpos_p3p1_x_k*dpos_p3p1_x_k + dpos_p3p1_y_k*dpos_p3p1_y_k + dpos_p3p1_z_k*dpos_p3p1_z_k);
    double proj_p3p1_x_ki = dpos_p3p1_x_k*relproj_p3p1_ki_mag + pos_p3_x_k;
    double proj_p3p1_y_ki = dpos_p3p1_y_k*relproj_p3p1_ki_mag + pos_p3_y_k;
    double proj_p3p1_z_ki = dpos_p3p1_z_k*relproj_p3p1_ki_mag + pos_p3_z_k;
    double dist_p3p1_ki_mag = sqrt((pos_x_i - proj_p3p1_x_ki)*(pos_x_i - proj_p3p1_x_ki) + (pos_y_i - proj_p3p1_y_ki)*(pos_y_i - proj_p3p1_y_ki) + (pos_z_i - proj_p3p1_z_ki)*(pos_z_i - proj_p3p1_z_ki));

    // calculate criteria for edge collision
    bool is_edge_p1p2_collision = dist_p1p2_ki_mag + TOLERANCE < rad_i;
    bool is_edge_p2p3_collision = dist_p2p3_ki_mag + TOLERANCE < rad_i;
    bool is_edge_p3p1_collision = dist_p3p1_ki_mag + TOLERANCE < rad_i;
    is_edge_collision = is_edge_p1p2_collision || is_edge_p2p3_collision || is_edge_p3p1_collision;

    // modify contact point if edge collision
    if(is_edge_p1p2_collision)
    {
        pos_contact_x = proj_p1p2_x_ki;
        pos_contact_y = proj_p1p2_y_ki;
        pos_contact_z = proj_p1p2_z_ki;
    }
    else if(is_edge_p2p3_collision)
    {
        pos_contact_x = proj_p2p3_x_ki;
        pos_contact_y = proj_p2p3_y_ki;
        pos_contact_z = proj_p2p3_z_ki;
    }
    else if(is_edge_p3p1_collision)
    {
        pos_contact_x = proj_p3p1_x_ki;
        pos_contact_y = proj_p3p1_y_ki;
        pos_contact_z = proj_p3p1_z_ki;
    }
    
}

void OutputForceForceSphereWallMeshHertz::check_vertex_collision(
    bool &is_vertex_collision,
    double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

    // get particle radius
    double rad_i = radius_vec[type_i];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];  

    // get wall points
    double pos_p1_x_k = wallmesh_pvs.position_p1_x_vec[indx_k];
    double pos_p1_y_k = wallmesh_pvs.position_p1_y_vec[indx_k];
    double pos_p1_z_k = wallmesh_pvs.position_p1_z_vec[indx_k];
    double pos_p2_x_k = wallmesh_pvs.position_p2_x_vec[indx_k];
    double pos_p2_y_k = wallmesh_pvs.position_p2_y_vec[indx_k];
    double pos_p2_z_k = wallmesh_pvs.position_p2_z_vec[indx_k];
    double pos_p3_x_k = wallmesh_pvs.position_p3_x_vec[indx_k];
    double pos_p3_y_k = wallmesh_pvs.position_p3_y_vec[indx_k];
    double pos_p3_z_k = wallmesh_pvs.position_p3_z_vec[indx_k];

    // calculate distance from P1 to center
    double dpos_p1_x_ik = -pos_x_i + pos_p1_x_k;
    double dpos_p1_y_ik = -pos_y_i + pos_p1_y_k;
    double dpos_p1_z_ik = -pos_z_i + pos_p1_z_k;
    double dpos_p1_ik_mag = sqrt(dpos_p1_x_ik*dpos_p1_x_ik + dpos_p1_y_ik*dpos_p1_y_ik + dpos_p1_z_ik*dpos_p1_z_ik);

    // calculate distance from P2 to center
    double dpos_p2_x_ik = -pos_x_i + pos_p2_x_k;
    double dpos_p2_y_ik = -pos_y_i + pos_p2_y_k;
    double dpos_p2_z_ik = -pos_z_i + pos_p2_z_k;
    double dpos_p2_ik_mag = sqrt(dpos_p2_x_ik*dpos_p2_x_ik + dpos_p2_y_ik*dpos_p2_y_ik + dpos_p2_z_ik*dpos_p2_z_ik);

    // calculate distance from P3 to center
    double dpos_p3_x_ik = -pos_x_i + pos_p3_x_k;
    double dpos_p3_y_ik = -pos_y_i + pos_p3_y_k;
    double dpos_p3_z_ik = -pos_z_i + pos_p3_z_k;
    double dpos_p3_ik_mag = sqrt(dpos_p3_x_ik*dpos_p3_x_ik + dpos_p3_y_ik*dpos_p3_y_ik + dpos_p3_z_ik*dpos_p3_z_ik);

    // calculate criteria for vertex collision
    bool is_vertex_p1_collision = dpos_p1_ik_mag < rad_i;
    bool is_vertex_p2_collision = dpos_p2_ik_mag < rad_i;
    bool is_vertex_p3_collision = dpos_p3_ik_mag < rad_i;
    is_vertex_collision = is_vertex_p1_collision || is_vertex_p2_collision || is_vertex_p3_collision;

    // modify contact point if edge collision
    if(is_vertex_p1_collision)
    {
        pos_contact_x = pos_p1_x_k;
        pos_contact_y = pos_p1_y_k;
        pos_contact_z = pos_p1_z_k;
    }
    else if(is_vertex_p2_collision)
    {
        pos_contact_x = pos_p2_x_k;
        pos_contact_y = pos_p2_y_k;
        pos_contact_z = pos_p2_z_k;
    }
    else if(is_vertex_p3_collision)
    {
        pos_contact_x = pos_p3_x_k;
        pos_contact_y = pos_p3_y_k;
        pos_contact_z = pos_p3_z_k;
    }
    
}

void OutputForceForceSphereWallMeshHertz::calculate_velocity_contact(
    double &vel_contact_x, double &vel_contact_y, double &vel_contact_z,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    double pos_contact_x, double pos_contact_y, double pos_contact_z
)
{

    // get wall velocities
    double vel_translate_x = wallmesh_pvs.velocity_translate_x;
    double vel_translate_y = wallmesh_pvs.velocity_translate_y;
    double vel_translate_z = wallmesh_pvs.velocity_translate_z;
    double angvel_rotate = wallmesh_pvs.angularvelocity_rotate;

    // get rotation axis points
    double axis_rotate_p1_x = wallmesh_pvs.axis_rotate_p1_x;
    double axis_rotate_p1_y = wallmesh_pvs.axis_rotate_p1_y;
    double axis_rotate_p1_z = wallmesh_pvs.axis_rotate_p1_z;
    double axis_rotate_p2_x = wallmesh_pvs.axis_rotate_p2_x;
    double axis_rotate_p2_y = wallmesh_pvs.axis_rotate_p2_y;
    double axis_rotate_p2_z = wallmesh_pvs.axis_rotate_p2_z;

    // axis of rotation is specified using two points: axis P1 and axis P2
    // calculate vector from axis P1 to axis P2 (labeled A1 and A2)
    double a1_a2_x = -axis_rotate_p1_x + axis_rotate_p2_x;
    double a1_a2_y = -axis_rotate_p1_y + axis_rotate_p2_y;
    double a1_a2_z = -axis_rotate_p1_z + axis_rotate_p2_z;

    // normalize vector along axis of rotation
    double helpvar_01 = 1./sqrt(a1_a2_x*a1_a2_x + a1_a2_y*a1_a2_y + a1_a2_z*a1_a2_z);
    double unit_axis_rotate_x = a1_a2_x*helpvar_01;
    double unit_axis_rotate_y = a1_a2_y*helpvar_01;
    double unit_axis_rotate_z = a1_a2_z*helpvar_01;
    if (abs(a1_a2_x) < TOLERANCE && abs(a1_a2_y) < TOLERANCE && abs(a1_a2_z) < TOLERANCE)
    {
        unit_axis_rotate_x = 0.;
        unit_axis_rotate_y = 0.;
        unit_axis_rotate_z = 0.;
    }

    // calculate vector from axis P1 to contact point
    double a1_cont_x = -axis_rotate_p1_x + pos_contact_x;
    double a1_cont_y = -axis_rotate_p1_y + pos_contact_y;
    double a1_cont_z = -axis_rotate_p1_z + pos_contact_z;

    // calculate distance from axis P1 to closest point
    // closest point - point along axis closest to contact point
    double a1_close_mag = a1_cont_x*unit_axis_rotate_x + a1_cont_y*unit_axis_rotate_y + a1_cont_z*unit_axis_rotate_z;

    // calculate vector from axis P1 to closest point
    double a1_close_x = a1_close_mag*unit_axis_rotate_x;
    double a1_close_y = a1_close_mag*unit_axis_rotate_y;
    double a1_close_z = a1_close_mag*unit_axis_rotate_z;

    // calculate vector from closest point to contact point
    double close_cont_x = -a1_close_x + a1_cont_x;
    double close_cont_y = -a1_close_y + a1_cont_y;
    double close_cont_z = -a1_close_z + a1_cont_z;

    // calculate rotational velocity at contact point
    double vel_rotate_x = angvel_rotate*(-close_cont_y*unit_axis_rotate_z + close_cont_z*unit_axis_rotate_y);
    double vel_rotate_y = angvel_rotate*(close_cont_x*unit_axis_rotate_z - close_cont_z*unit_axis_rotate_x);
    double vel_rotate_z = angvel_rotate*(-close_cont_x*unit_axis_rotate_y + close_cont_y*unit_axis_rotate_x);

    // calculate velocity at contact point
    vel_contact_x = vel_translate_x + vel_rotate_x;
    vel_contact_y = vel_translate_y + vel_rotate_y;
    vel_contact_z = vel_translate_z + vel_rotate_z;

}

#endif
