#ifndef OUTPUT_SPHEREWALLMESH_FORCEMOMENT_HERTZ
#define OUTPUT_SPHEREWALLMESH_FORCEMOMENT_HERTZ
#include <fstream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>
#include "collisioncheck_spherewallmesh_naive.hpp"
#include "collisioncheck_spherewallmesh_sweep_1dx.hpp"
#include "collisioncheck_spherewallmesh_sweep_1dy.hpp"
#include "collisioncheck_spherewallmesh_sweep_1dz.hpp"
#include "container_smat_integrable.hpp"
#include "container_sphere.hpp"
#include "container_typedef.hpp"
#include "container_wallmesh.hpp"

template <class CollisionCheckSphereWallMesh>
class OutputSphereWallMeshForceMomentHertz
{
    /*

    Calculates collision forces and moments between spheres and mesh triangles.
    Uses the Hertzian (spring-dashpot) model to calculate collision forces.
    Outputs additional statistics about the collisions.

    Variables
    =========
    radius_vec_in : VectorDouble
        vector with the radius of each type of sphere.
    springconstant_normal_mat_in : MatrixDouble
        Matrix (nested vector) with the normal spring constant of each type of sphere-wallmesh interaction.
    springconstant_tangent_mat_in : MatrixDouble
        Matrix with the tangential spring constant of each type of sphere-wallmesh interaction.
    dampingcoefficient_normal_mat_in : MatrixDouble
        Matrix with the normal damping coefficient of each type of sphere-wallmesh interaction.
    dampingcoefficient_tangent_mat_in : MatrixDouble
        Matrix with the tangential damping coefficient of each type of sphere-wallmesh interaction.
    frictioncoefficient_sliding_mat_in : MatrixDouble
        Matrix with the sliding friction coefficient of each type of sphere-wallmesh interaction.
    frictioncoefficient_rolling_mat_in : MatrixDouble
        Matrix with the rolling friction coefficient of each type of sphere-wallmesh interaction.
    file_precollision_positionvelocity_str_in : string
        File name where particle velocities (prior to collision) will be stored.
    file_wallmesh_forcemoment_str_in : string
        File name where total force and moment (with respect to a reference point) will be stored.
    pos_reference_x_in : double
        x-coordinate of the reference point.
    pos_reference_y_in : double
        y-coordinate of the reference point.
    pos_reference_z_in : double
        z-coordinate of the reference point.

    Functions
    =========
    add_forcemoment : void
        Adds forces and moments to spheres in the simulation.

    Notes
    =====
    Use this class instead of ForceMomentSphereWallMeshHertz if additional statistics about the collisions are to be outputted.
    Set the file name to an empty string "" if no file is to be generated.

    */

    public:

    // constant
    double TOLERANCE = 1e-10;

    // variables
    VectorDouble radius_vec;
    MatrixDouble springconstant_normal_mat;
    MatrixDouble springconstant_tangent_mat;
    MatrixDouble dampingcoefficient_normal_mat;
    MatrixDouble dampingcoefficient_tangent_mat;
    MatrixDouble frictioncoefficient_sliding_mat;
    MatrixDouble frictioncoefficient_rolling_mat;

    // output files - pre-collision position and velocity
    bool write_precollision_positionvelocity;
    std::string file_precollision_positionvelocity_str;
    std::ofstream file_precollision_positionvelocity_stream;

    // output files - wallmesh force and moment
    bool write_wallmesh_forcemoment;
    std::string file_wallmesh_forcemoment_str;
    std::ofstream file_wallmesh_forcemoment_stream;
    double pos_reference_x;
    double pos_reference_y;
    double pos_reference_z;

    // collision checker
    CollisionCheckSphereWallMesh collision_check;

    // functions
    void add_forcemoment(
        SphereForceMomentStruct &sphere_fms,
        SparseMatrixIntegrable &overlap_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int ts
    );

    // constructor
    OutputSphereWallMeshForceMomentHertz(
        VectorDouble radius_vec_in,
        MatrixDouble springconstant_normal_mat_in,
        MatrixDouble springconstant_tangent_mat_in,
        MatrixDouble dampingcoefficient_normal_mat_in,
        MatrixDouble dampingcoefficient_tangent_mat_in,
        MatrixDouble frictioncoefficient_sliding_mat_in,
        MatrixDouble frictioncoefficient_rolling_mat_in,
        std::string file_precollision_positionvelocity_str_in,
        std::string file_wallmesh_forcemoment_str_in,
        double pos_reference_x_in,
        double pos_reference_y_in,
        double pos_reference_z_in
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
        file_precollision_positionvelocity_str = file_precollision_positionvelocity_str_in;
        file_wallmesh_forcemoment_str = file_wallmesh_forcemoment_str_in;
        pos_reference_x_in = pos_reference_x;
        pos_reference_y_in = pos_reference_y;
        pos_reference_z_in = pos_reference_z;

        // initialize collision checker
        collision_check.set_input(radius_vec);

        // initialize output file - precollision position and velocity
        write_precollision_positionvelocity = !(file_precollision_positionvelocity_str == "");  // "" -> no output
        file_precollision_positionvelocity_stream.open(file_precollision_positionvelocity_str);
        file_precollision_positionvelocity_stream << "ts,id_i,id_k,type_i,type_k,pos_x_i,pos_y_i,pos_z_i,pos_x_k,pos_y_k,pos_z_k,vel_x_i,vel_y_i,vel_z_i,vel_x_k,vel_y_k,vel_z_k,angpos_x_i,angpos_y_i,angpos_z_i,angvel_x_i,angvel_y_i,angvel_z_i\n";

        // initialize output file - wallmesh force and moment
        write_wallmesh_forcemoment = !(file_wallmesh_forcemoment_str == "");  // "" -> no output
        file_wallmesh_forcemoment_stream.open(file_wallmesh_forcemoment_str);
        file_wallmesh_forcemoment_stream << "ts,id_i,id_k,type_i,type_k,pos_x_i,pos_y_i,pos_z_i,pos_x_k,pos_y_k,pos_z_k,vel_x_i,vel_y_i,vel_z_i,vel_x_k,vel_y_k,vel_z_k,angpos_x_i,angpos_y_i,angpos_z_i,angvel_x_i,angvel_y_i,angvel_z_i\n";

    }

    private:
    void calculate_forcemoment(
        SphereForceMomentStruct &sphere_fms,
        SphereForceMomentStruct &wallmesh_fms,
        SparseMatrixIntegrable &overlap_tangent_smat,
        SpherePositionVelocityStruct &sphere_pvs,
        WallMeshPositionVelocityStruct &wallmesh_pvs,
        int indx_i, int indx_k, int ts
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

template <class CollisionCheckSphereWallMesh>
void OutputSphereWallMeshForceMomentHertz<CollisionCheckSphereWallMesh>::add_forcemoment(
    SphereForceMomentStruct &sphere_fms,
    SparseMatrixIntegrable &overlap_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int ts
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
    wallmesh_pvs : SphereParticleVelocityStruct
        struct with position and velocity of the wall.
    ts : int
        nth timestep in the simulation.

    Returns
    =======
    (none)

    */

    // initialize forces and moments on wallmesh
    SphereForceMomentStruct wallmesh_fms = sphere_fms_fill(sphere_fms.num_particle);

    // generate preliminary list of collision pairs
    VectorPairInt collision_vec = collision_check.broad_search(sphere_pvs, wallmesh_pvs);

    // calculate force for each collision pair
    for (auto &collision_pair : collision_vec)
    {
        
        // get indices of particles
        int indx_i = collision_pair.first;
        int indx_k = collision_pair.second;

        // calculate forces
        calculate_forcemoment(sphere_fms, wallmesh_fms, overlap_tangent_smat, sphere_pvs, wallmesh_pvs, indx_i, indx_k, ts);
        
    }

    // write data on forces on wallmesh
    if (write_wallmesh_forcemoment)
    {

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
        file_wallmesh_forcemoment_stream << ts << ",";
        file_wallmesh_forcemoment_stream << frc_x_ki << ",";
        file_wallmesh_forcemoment_stream << frc_y_ki << ",";
        file_wallmesh_forcemoment_stream << frc_z_ki << ",";
        file_wallmesh_forcemoment_stream << mom_x_ki << ",";
        file_wallmesh_forcemoment_stream << mom_y_ki << ",";
        file_wallmesh_forcemoment_stream << mom_z_ki << "\n";

    }

}

template <class CollisionCheckSphereWallMesh>
void OutputSphereWallMeshForceMomentHertz<CollisionCheckSphereWallMesh>::calculate_forcemoment(
    SphereForceMomentStruct &sphere_fms,
    SphereForceMomentStruct &wallmesh_fms,
    SparseMatrixIntegrable &overlap_tangent_smat,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k, int ts
)
{
    /*

    Calculates forces and moments acting on spheres in sphere-wallmesh collisions.

    */

    // get particle and wall id
    int id_i = sphere_pvs.id_vec[indx_i];
    int id_k = wallmesh_pvs.id_vec[indx_k];

    // skip if too far for collision
    bool is_possible_collision = check_possible_collision(sphere_pvs, wallmesh_pvs, indx_i, indx_k);
    if (!is_possible_collision)
    {
        smat_prune(overlap_tangent_smat.u, id_i, id_k);  // reset tangential overlap
        smat_prune(overlap_tangent_smat.dudt, id_i, id_k);
        return;
    }

    // initialize contact point
    double pos_contact_x_ik = 0.;
    double pos_contact_y_ik = 0.;
    double pos_contact_z_ik = 0.;

    // initialize collision type indicator
    bool is_face_collision = false;
    bool is_edge_collision = false;
    bool is_vertex_collision = false;

    // check if face collision
    check_face_collision(is_face_collision, pos_contact_x_ik, pos_contact_y_ik, pos_contact_z_ik, sphere_pvs, wallmesh_pvs, indx_i, indx_k);
    if (!is_face_collision)
    {

        // check if edge collision
        check_edge_collision(is_edge_collision, pos_contact_x_ik, pos_contact_y_ik, pos_contact_z_ik, sphere_pvs, wallmesh_pvs, indx_i, indx_k);
        if (!is_edge_collision)
        {
            
            // check if vertex collision
            check_vertex_collision(is_vertex_collision, pos_contact_x_ik, pos_contact_y_ik, pos_contact_z_ik, sphere_pvs, wallmesh_pvs, indx_i, indx_k);
            
            // skip if no collision
            if (!is_vertex_collision)
            {
                smat_prune(overlap_tangent_smat.u, id_i, id_k);  // reset tangential overlap
                smat_prune(overlap_tangent_smat.dudt, id_i, id_k);
                return;
            }

        }

    }

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

    // get particle radius
    double radius_i = radius_vec[type_i];

    // get particle positions
    double pos_x_i = sphere_pvs.position_x_vec[indx_i];
    double pos_y_i = sphere_pvs.position_y_vec[indx_i];
    double pos_z_i = sphere_pvs.position_z_vec[indx_i];  

    // calculate displacement from center of i to contact point on k
    double delta_pos_x_ik = -pos_x_i + pos_contact_x_ik;
    double delta_pos_y_ik = -pos_y_i + pos_contact_y_ik;
    double delta_pos_z_ik = -pos_z_i + pos_contact_z_ik;

    // calculate distance from center of i to contact point on k
    double delta_pos_ik_mag = sqrt(delta_pos_x_ik*delta_pos_x_ik + delta_pos_y_ik*delta_pos_y_ik + delta_pos_z_ik*delta_pos_z_ik);

    // calculate normal vector
    double normal_x_ik = delta_pos_x_ik/delta_pos_ik_mag;
    double normal_y_ik = delta_pos_y_ik/delta_pos_ik_mag;
    double normal_z_ik = delta_pos_z_ik/delta_pos_ik_mag;

    // calculate velocity at contact point
    double vel_x_k = 0;
    double vel_y_k = 0;
    double vel_z_k = 0;
    calculate_velocity_contact(vel_x_k, vel_y_k, vel_z_k, wallmesh_pvs, pos_contact_x_ik, pos_contact_y_ik, pos_contact_z_ik);

    // get particle velocities
    double vel_x_i = sphere_pvs.velocity_x_vec[indx_i];
    double vel_y_i = sphere_pvs.velocity_y_vec[indx_i];
    double vel_z_i = sphere_pvs.velocity_z_vec[indx_i];

    // get particle angular velocities
    double angvel_x_i = sphere_pvs.angularvelocity_x_vec[indx_i];
    double angvel_y_i = sphere_pvs.angularvelocity_y_vec[indx_i];
    double angvel_z_i = sphere_pvs.angularvelocity_z_vec[indx_i];

    // get collision properties
    double springconstant_normal = springconstant_normal_mat[type_i][type_k];
    double springconstant_tangent = springconstant_tangent_mat[type_i][type_k];
    double dampingcoefficient_normal = dampingcoefficient_normal_mat[type_i][type_k];
    double dampingcoefficient_tangent = dampingcoefficient_tangent_mat[type_i][type_k];
    double frictioncoefficient_sliding = frictioncoefficient_sliding_mat[type_i][type_k];
    double frictioncoefficient_rolling = frictioncoefficient_rolling_mat[type_i][type_k];

    // get tangential overlap
    double overlap_tangent_ik_val = smat_get_value(overlap_tangent_smat.u, id_i, id_k);

    // write data on collision pair if no collision on previous timestep
    if (write_precollision_positionvelocity && overlap_tangent_ik_val == 0.)
    {
        
        // get particle angular position
        double angpos_x_i = sphere_pvs.angularposition_x_vec[indx_i];
        double angpos_y_i = sphere_pvs.angularposition_y_vec[indx_i];
        double angpos_z_i = sphere_pvs.angularposition_z_vec[indx_i];

        // write to file
        file_precollision_positionvelocity_stream << ts << ",";
        file_precollision_positionvelocity_stream << id_i << ",";
        file_precollision_positionvelocity_stream << id_k << ",";
        file_precollision_positionvelocity_stream << type_i << ",";
        file_precollision_positionvelocity_stream << type_k << ",";
        file_precollision_positionvelocity_stream << pos_x_i << ",";
        file_precollision_positionvelocity_stream << pos_y_i << ",";
        file_precollision_positionvelocity_stream << pos_z_i << ",";
        file_precollision_positionvelocity_stream << pos_contact_x_ik << ",";
        file_precollision_positionvelocity_stream << pos_contact_y_ik << ",";
        file_precollision_positionvelocity_stream << pos_contact_z_ik << ",";
        file_precollision_positionvelocity_stream << vel_x_i << ",";
        file_precollision_positionvelocity_stream << vel_y_i << ",";
        file_precollision_positionvelocity_stream << vel_z_i << ",";
        file_precollision_positionvelocity_stream << vel_x_k << ",";
        file_precollision_positionvelocity_stream << vel_y_k << ",";
        file_precollision_positionvelocity_stream << vel_z_k << ",";
        file_precollision_positionvelocity_stream << angpos_x_i << ",";
        file_precollision_positionvelocity_stream << angpos_y_i << ",";
        file_precollision_positionvelocity_stream << angpos_z_i << ",";
        file_precollision_positionvelocity_stream << angvel_x_i << ",";
        file_precollision_positionvelocity_stream << angvel_y_i << ",";
        file_precollision_positionvelocity_stream << angvel_z_i << "\n";

    }

    // calculate relative velocity vector
    double relvel_x_ik = -normal_y_ik*(angvel_z_i*radius_i) + normal_z_ik*(angvel_y_i*radius_i) + vel_x_i - vel_x_k;
    double relvel_y_ik =  normal_x_ik*(angvel_z_i*radius_i) - normal_z_ik*(angvel_x_i*radius_i) + vel_y_i - vel_y_k;
    double relvel_z_ik = -normal_x_ik*(angvel_y_i*radius_i) + normal_y_ik*(angvel_x_i*radius_i) + vel_z_i - vel_z_k;

    // calculate magnitude of normal component of relative velocity
    double relvel_normal_ik_val = normal_x_ik*relvel_x_ik + normal_y_ik*relvel_y_ik + normal_z_ik*relvel_z_ik;

    // calculate normal component of relative velocity
    double relvel_normal_x_ik = normal_x_ik*relvel_normal_ik_val;
    double relvel_normal_y_ik = normal_y_ik*relvel_normal_ik_val;
    double relvel_normal_z_ik = normal_z_ik*relvel_normal_ik_val;

    // calculate tangential component of relative velocity
    double relvel_tangent_x_ik = -relvel_normal_x_ik + relvel_x_ik;
    double relvel_tangent_y_ik = -relvel_normal_y_ik + relvel_y_ik;
    double relvel_tangent_z_ik = -relvel_normal_z_ik + relvel_z_ik;

    // calculate tangential vector
    // tangential vector is zero if tangential component of relative velocity is zero
    double helpvar_02 = 1./sqrt(relvel_tangent_x_ik*relvel_tangent_x_ik + relvel_tangent_y_ik*relvel_tangent_y_ik + relvel_tangent_z_ik*relvel_tangent_z_ik);
    double tangent_x_ik = relvel_tangent_x_ik*helpvar_02;
    double tangent_y_ik = relvel_tangent_y_ik*helpvar_02;
    double tangent_z_ik = relvel_tangent_z_ik*helpvar_02;
    if (relvel_tangent_x_ik == 0. && relvel_tangent_y_ik == 0. && relvel_tangent_z_ik == 0.)
    {
        tangent_x_ik = 0.;
        tangent_y_ik = 0.;
        tangent_z_ik = 0.;
    }

    // calculate tangential component of relative velocity
    double relvel_tangent_ik_val = relvel_x_ik*tangent_x_ik + relvel_y_ik*tangent_y_ik + relvel_z_ik*tangent_z_ik;

    // calculate normal overlap
    double overlap_normal_ik_val = radius_i - delta_pos_ik_mag;

    // calculate magnitude of normal component of collision force
    double force_collision_normal_ik_val = -dampingcoefficient_normal*relvel_normal_ik_val - overlap_normal_ik_val*springconstant_normal;
    double force_collision_normal_ik_mag = abs(force_collision_normal_ik_val);

    // calculate normal component of collision force
    double force_collision_normal_x_ik = force_collision_normal_ik_val*normal_x_ik;
    double force_collision_normal_y_ik = force_collision_normal_ik_val*normal_y_ik;
    double force_collision_normal_z_ik = force_collision_normal_ik_val*normal_z_ik;

    // calculate magnitude of tangential component of collision force
    double force_collision_tangent_ik_val = -dampingcoefficient_tangent*relvel_tangent_ik_val - overlap_tangent_ik_val*springconstant_tangent;
    double force_collision_tangent_ik_mag = abs(force_collision_tangent_ik_val);

    // recalculate tangential force if it exceeds maximum static friction
    if (force_collision_tangent_ik_mag > frictioncoefficient_sliding*force_collision_normal_ik_mag)
    {
        
        // calcualte signum of tangential overlap
        double signum_overlap_tangent_ik_val = 1.;
        if (overlap_tangent_ik_val < 0.)
        {
            signum_overlap_tangent_ik_val = -1.;
        }
        
        // calculate magnitude of tangential component of collision force (friction)
        force_collision_tangent_ik_val = -force_collision_normal_ik_mag*frictioncoefficient_sliding*signum_overlap_tangent_ik_val;

    }

    // calculate tangential component of collision force
    double force_collision_tangent_x_ik = force_collision_tangent_ik_val*tangent_x_ik;
    double force_collision_tangent_y_ik = force_collision_tangent_ik_val*tangent_y_ik;
    double force_collision_tangent_z_ik = force_collision_tangent_ik_val*tangent_z_ik;

    // calculate collision force
    double force_collision_x_ik = force_collision_normal_x_ik + force_collision_tangent_x_ik;
    double force_collision_y_ik = force_collision_normal_y_ik + force_collision_tangent_y_ik;
    double force_collision_z_ik = force_collision_normal_z_ik + force_collision_tangent_z_ik;

    // update collision matrix
    smat_set_value(overlap_tangent_smat.dudt, id_i, id_k, relvel_tangent_ik_val);

    // face collision -> add to total force
    // edge or vertex collision -> add to average force
    if (is_face_collision)
    {
        
        // forces on spheres
        sphere_fms.force_sum_x_vec[indx_i] += force_collision_x_ik;
        sphere_fms.force_sum_y_vec[indx_i] += force_collision_y_ik;
        sphere_fms.force_sum_z_vec[indx_i] += force_collision_z_ik;

        // forces on wallmesh
        wallmesh_fms.force_sum_x_vec[indx_i] += -force_collision_x_ik;
        wallmesh_fms.force_sum_y_vec[indx_i] += -force_collision_y_ik;
        wallmesh_fms.force_sum_z_vec[indx_i] += -force_collision_z_ik;

    }
    else if (is_edge_collision || is_vertex_collision)
    {
        
        // forces on spheres
        sphere_fms.force_average_x_vec[indx_i] += force_collision_x_ik;
        sphere_fms.force_average_y_vec[indx_i] += force_collision_y_ik;
        sphere_fms.force_average_z_vec[indx_i] += force_collision_z_ik;

        // forces on wallmesh
        wallmesh_fms.force_average_x_vec[indx_i] += -force_collision_x_ik;
        wallmesh_fms.force_average_y_vec[indx_i] += -force_collision_y_ik;
        wallmesh_fms.force_average_z_vec[indx_i] += -force_collision_z_ik;

        // increment contact count
        sphere_fms.num_contact_vec[indx_i]++;

    }

    // calculate collision moment
    double moment_collision_x_ik = radius_i*(-force_collision_tangent_y_ik*normal_z_ik + force_collision_tangent_z_ik*normal_y_ik);
    double moment_collision_y_ik = radius_i*( force_collision_tangent_x_ik*normal_z_ik - force_collision_tangent_z_ik*normal_x_ik);
    double moment_collision_z_ik = radius_i*(-force_collision_tangent_x_ik*normal_y_ik + force_collision_tangent_y_ik*normal_x_ik);

    // calculate relative angular velocities of i and k
    double relangvel_x_ik = angvel_x_i;
    double relangvel_y_ik = angvel_y_i;
    double relangvel_z_ik = angvel_z_i;

    // calculate unit vector pointing to relative angular velocity
    // unit vector is zero if difference between angular velocities is zero
    double helpvar_03 = 1./sqrt(relangvel_x_ik*relangvel_x_ik + relangvel_y_ik*relangvel_y_ik + relangvel_z_ik*relangvel_z_ik);
    double axis_x_ik = relangvel_x_ik*helpvar_03;
    double axis_y_ik = relangvel_y_ik*helpvar_03;
    double axis_z_ik = relangvel_z_ik*helpvar_03;
    if (relangvel_x_ik == 0. && relangvel_y_ik == 0. && relangvel_z_ik == 0.)
    {
        axis_x_ik = 0.;
        axis_y_ik = 0.;
        axis_z_ik = 0.;
    }

    // calculate effective radius
    double radius_effective = radius_i;

    // calculate magnitude of fricion moment
    double moment_friction_ik_val = -force_collision_normal_ik_mag*frictioncoefficient_rolling*radius_effective;

    // calculate friction moment on i
    double moment_friction_x_ik = moment_friction_ik_val*axis_x_ik;
    double moment_friction_y_ik = moment_friction_ik_val*axis_y_ik;
    double moment_friction_z_ik = moment_friction_ik_val*axis_z_ik;

    // get lever arm from reference point to particle
    double delta_pos_reference_contact_x = pos_contact_x_ik - pos_reference_x;
    double delta_pos_reference_contact_y = pos_contact_y_ik - pos_reference_y;
    double delta_pos_reference_contact_z = pos_contact_z_ik - pos_reference_z;

    // face collision -> add to total moment
    // edge or vertex collision -> add to average monent
    if (is_face_collision)
    {
        
        // moment on spheres
        sphere_fms.moment_sum_x_vec[indx_i] += moment_collision_x_ik + moment_friction_x_ik;
        sphere_fms.moment_sum_y_vec[indx_i] += moment_collision_y_ik + moment_friction_y_ik;
        sphere_fms.moment_sum_z_vec[indx_i] += moment_collision_z_ik + moment_friction_z_ik;

        // moment on wallmesh
        wallmesh_fms.moment_sum_x_vec[indx_i] -= +delta_pos_reference_contact_y*force_collision_z_ik - delta_pos_reference_contact_z*force_collision_y_ik;
        wallmesh_fms.moment_sum_y_vec[indx_i] -= -delta_pos_reference_contact_x*force_collision_z_ik + delta_pos_reference_contact_z*force_collision_x_ik;
        wallmesh_fms.moment_sum_z_vec[indx_i] -= +delta_pos_reference_contact_x*force_collision_y_ik - delta_pos_reference_contact_y*force_collision_x_ik;

    }
    else if (is_edge_collision || is_vertex_collision)
    {
        
        // moment on spheres
        sphere_fms.moment_average_x_vec[indx_i] += moment_collision_x_ik + moment_friction_x_ik;
        sphere_fms.moment_average_y_vec[indx_i] += moment_collision_y_ik + moment_friction_y_ik;
        sphere_fms.moment_average_z_vec[indx_i] += moment_collision_z_ik + moment_friction_z_ik;
        
        // moment on wallmesh
        wallmesh_fms.moment_sum_x_vec[indx_i] -= +delta_pos_reference_contact_y*force_collision_z_ik - delta_pos_reference_contact_z*force_collision_y_ik;
        wallmesh_fms.moment_sum_y_vec[indx_i] -= -delta_pos_reference_contact_x*force_collision_z_ik + delta_pos_reference_contact_z*force_collision_x_ik;
        wallmesh_fms.moment_sum_z_vec[indx_i] -= +delta_pos_reference_contact_x*force_collision_y_ik - delta_pos_reference_contact_y*force_collision_x_ik;

    }

}

template <class CollisionCheckSphereWallMesh>
bool OutputSphereWallMeshForceMomentHertz<CollisionCheckSphereWallMesh>::check_possible_collision(
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{
    /*

    Checks if center of sphere is too far from mesh triangle.

    */

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

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

    // get particle radius
    double radius_i = radius_vec[type_i];

    // calculate vector facing the "front" of the triangle
    double front_x_ki =  (pos_p1_y_k - pos_p2_y_k)*(pos_p1_z_k - pos_p3_z_k) - (pos_p1_y_k - pos_p3_y_k)*(pos_p1_z_k - pos_p2_z_k);
    double front_y_ki = -(pos_p1_x_k - pos_p2_x_k)*(pos_p1_z_k - pos_p3_z_k) + (pos_p1_x_k - pos_p3_x_k)*(pos_p1_z_k - pos_p2_z_k);
    double front_z_ki =  (pos_p1_x_k - pos_p2_x_k)*(pos_p1_y_k - pos_p3_y_k) - (pos_p1_x_k - pos_p3_x_k)*(pos_p1_y_k - pos_p2_y_k);

    // calculate front facing normal vector
    double helpvar_01 = 1./sqrt(front_x_ki*front_x_ki + front_y_ki*front_y_ki + front_z_ki*front_z_ki);
    double norm_front_x_ki = front_x_ki*helpvar_01;
    double norm_front_y_ki = front_y_ki*helpvar_01;
    double norm_front_z_ki = front_z_ki*helpvar_01;

    // calculate back facing normal vector
    double norm_back_x_ki = -norm_front_x_ki;
    double norm_back_y_ki = -norm_front_y_ki;
    double norm_back_z_ki = -norm_front_z_ki;

    // calculate front facing wall to particle distance
    double dist_front_ki_val = -norm_front_x_ki*(pos_p1_x_k - pos_x_i) - norm_front_y_ki*(pos_p1_y_k - pos_y_i) - norm_front_z_ki*(pos_p1_z_k - pos_z_i);

    // calculate back facing wall to particle distance
    double dist_back_ki_val = -dist_front_ki_val;

    // determine whether to use front or back facing normal and distance
    // particle faces the front if the distance from the front is positive
    double dist_ki_val = 0.;
    double norm_x_ki = 0.;
    double norm_y_ki = 0.;
    double norm_z_ki = 0.;
    if (dist_front_ki_val >= 0)
    {
        dist_ki_val = dist_front_ki_val;
        norm_x_ki = norm_front_x_ki;
        norm_y_ki = norm_front_y_ki;
        norm_z_ki = norm_front_z_ki;
    }
    else
    {
        dist_ki_val = dist_back_ki_val;
        norm_x_ki = norm_back_x_ki;
        norm_y_ki = norm_back_y_ki;
        norm_z_ki = norm_back_z_ki;
    }

    // false if particle is too far for collision
    return dist_ki_val < radius_i;

}

template <class CollisionCheckSphereWallMesh>
void OutputSphereWallMeshForceMomentHertz<CollisionCheckSphereWallMesh>::check_face_collision(
    bool &is_face_collision, 
    double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{
    /*

    Checks if sphere collides with face of mesh triangle.
    If so, calculates the contact point.

    */

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

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

    // get particle radius
    double radius_i = radius_vec[type_i];

    // calculate vector facing the "front" of the triangle
    double front_x_ki =  (pos_p1_y_k - pos_p2_y_k)*(pos_p1_z_k - pos_p3_z_k) - (pos_p1_y_k - pos_p3_y_k)*(pos_p1_z_k - pos_p2_z_k);
    double front_y_ki = -(pos_p1_x_k - pos_p2_x_k)*(pos_p1_z_k - pos_p3_z_k) + (pos_p1_x_k - pos_p3_x_k)*(pos_p1_z_k - pos_p2_z_k);
    double front_z_ki =  (pos_p1_x_k - pos_p2_x_k)*(pos_p1_y_k - pos_p3_y_k) - (pos_p1_x_k - pos_p3_x_k)*(pos_p1_y_k - pos_p2_y_k);

    // calculate front facing normal vector
    double helpvar_01 = 1./sqrt(front_x_ki*front_x_ki + front_y_ki*front_y_ki + front_z_ki*front_z_ki);
    double normal_front_x_ki = front_x_ki*helpvar_01;
    double normal_front_y_ki = front_y_ki*helpvar_01;
    double normal_front_z_ki = front_z_ki*helpvar_01;

    // calculate back facing normal vector
    double normal_back_x_ki = -normal_front_x_ki;
    double normal_back_y_ki = -normal_front_y_ki;
    double normal_back_z_ki = -normal_front_z_ki;

    // calculate front facing wall to particle distance
    double dist_front_ki_val = -normal_front_x_ki*(pos_p1_x_k - pos_x_i) - normal_front_y_ki*(pos_p1_y_k - pos_y_i) - normal_front_z_ki*(pos_p1_z_k - pos_z_i);

    // calculate back facing wall to particle distance
    double dist_back_ki_val = -dist_front_ki_val;

    // determine whether to use front or back facing normal and distance
    // particle faces the front if the distance from the front is positive
    double dist_ki_val = 0.;
    double normal_x_ki = 0.;
    double normal_y_ki = 0.;
    double normal_z_ki = 0.;
    if (dist_front_ki_val >= 0)
    {
        dist_ki_val = dist_front_ki_val;
        normal_x_ki = normal_front_x_ki;
        normal_y_ki = normal_front_y_ki;
        normal_z_ki = normal_front_z_ki;
    }
    else
    {
        dist_ki_val = dist_back_ki_val;
        normal_x_ki = normal_back_x_ki;
        normal_y_ki = normal_back_y_ki;
        normal_z_ki = normal_back_z_ki;
    }

    // calculate preliminary contact point
    double preliminary_contact_x_ki = -dist_ki_val*normal_x_ki + pos_x_i;
    double preliminary_contact_y_ki = -dist_ki_val*normal_y_ki + pos_y_i;
    double preliminary_contact_z_ki = -dist_ki_val*normal_z_ki + pos_z_i;

    // calculate test vectors
    double facetest_vec1_x_ki =  (pos_p1_y_k - preliminary_contact_y_ki)*(pos_p2_z_k - preliminary_contact_z_ki) - (pos_p1_z_k - preliminary_contact_z_ki)*(pos_p2_y_k - preliminary_contact_y_ki);
    double facetest_vec1_y_ki = -(pos_p1_x_k - preliminary_contact_x_ki)*(pos_p2_z_k - preliminary_contact_z_ki) + (pos_p1_z_k - preliminary_contact_z_ki)*(pos_p2_x_k - preliminary_contact_x_ki);
    double facetest_vec1_z_ki =  (pos_p1_x_k - preliminary_contact_x_ki)*(pos_p2_y_k - preliminary_contact_y_ki) - (pos_p1_y_k - preliminary_contact_y_ki)*(pos_p2_x_k - preliminary_contact_x_ki);
    double facetest_vec2_x_ki =  (pos_p2_y_k - preliminary_contact_y_ki)*(pos_p3_z_k - preliminary_contact_z_ki) - (pos_p2_z_k - preliminary_contact_z_ki)*(pos_p3_y_k - preliminary_contact_y_ki);
    double facetest_vec2_y_ki = -(pos_p2_x_k - preliminary_contact_x_ki)*(pos_p3_z_k - preliminary_contact_z_ki) + (pos_p2_z_k - preliminary_contact_z_ki)*(pos_p3_x_k - preliminary_contact_x_ki);
    double facetest_vec2_z_ki =  (pos_p2_x_k - preliminary_contact_x_ki)*(pos_p3_y_k - preliminary_contact_y_ki) - (pos_p2_y_k - preliminary_contact_y_ki)*(pos_p3_x_k - preliminary_contact_x_ki);
    double facetest_vec3_x_ki = -(pos_p1_y_k - preliminary_contact_y_ki)*(pos_p3_z_k - preliminary_contact_z_ki) + (pos_p1_z_k - preliminary_contact_z_ki)*(pos_p3_y_k - preliminary_contact_y_ki);
    double facetest_vec3_y_ki =  (pos_p1_x_k - preliminary_contact_x_ki)*(pos_p3_z_k - preliminary_contact_z_ki) - (pos_p1_z_k - preliminary_contact_z_ki)*(pos_p3_x_k - preliminary_contact_x_ki);
    double facetest_vec3_z_ki = -(pos_p1_x_k - preliminary_contact_x_ki)*(pos_p3_y_k - preliminary_contact_y_ki) + (pos_p1_y_k - preliminary_contact_y_ki)*(pos_p3_x_k - preliminary_contact_x_ki);
    double facetest_criteria1_ki_val = facetest_vec1_x_ki*facetest_vec2_x_ki + facetest_vec1_y_ki*facetest_vec2_y_ki + facetest_vec1_z_ki*facetest_vec2_z_ki;
    double facetest_criteria2_ki_val = facetest_vec1_x_ki*facetest_vec3_x_ki + facetest_vec1_y_ki*facetest_vec3_y_ki + facetest_vec1_z_ki*facetest_vec3_z_ki;

    // calculate criteria for face collision
    is_face_collision = facetest_criteria1_ki_val > TOLERANCE && facetest_criteria2_ki_val > TOLERANCE;

    // record contact point
    if (is_face_collision)
    {
        pos_contact_x = preliminary_contact_x_ki;
        pos_contact_y = preliminary_contact_y_ki;
        pos_contact_z = preliminary_contact_z_ki;
    }

}

template <class CollisionCheckSphereWallMesh>
void OutputSphereWallMeshForceMomentHertz<CollisionCheckSphereWallMesh>::check_edge_collision(
    bool &is_edge_collision,
    double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{
    /*

    Checks if sphere collides with edge of mesh triangle.
    If so, calculates the contact point.

    */

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

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

    // get particle radius
    double radius_i = radius_vec[type_i];

    // calculate distance from edge P1 to P2 and center
    double delta_pos_p1p2_x_k = -pos_p1_x_k + pos_p2_x_k;
    double delta_pos_p1p2_y_k = -pos_p1_y_k + pos_p2_y_k;
    double delta_pos_p1p2_z_k = -pos_p1_z_k + pos_p2_z_k;
    double relprojection_p1p2_ki_val = (-delta_pos_p1p2_x_k*(pos_p1_x_k - pos_x_i) - delta_pos_p1p2_y_k*(pos_p1_y_k - pos_y_i) - delta_pos_p1p2_z_k*(pos_p1_z_k - pos_z_i))/(delta_pos_p1p2_x_k*delta_pos_p1p2_x_k + delta_pos_p1p2_y_k*delta_pos_p1p2_y_k + delta_pos_p1p2_z_k*delta_pos_p1p2_z_k);
    double projection_p1p2_x_ki = delta_pos_p1p2_x_k*relprojection_p1p2_ki_val + pos_p1_x_k;
    double projection_p1p2_y_ki = delta_pos_p1p2_y_k*relprojection_p1p2_ki_val + pos_p1_y_k;
    double projection_p1p2_z_ki = delta_pos_p1p2_z_k*relprojection_p1p2_ki_val + pos_p1_z_k;
    double dist_p1p2_ki_mag = sqrt((pos_x_i - projection_p1p2_x_ki)*(pos_x_i - projection_p1p2_x_ki) + (pos_y_i - projection_p1p2_y_ki)*(pos_y_i - projection_p1p2_y_ki) + (pos_z_i - projection_p1p2_z_ki)*(pos_z_i - projection_p1p2_z_ki));

    // calculate distance from edge P2 to P3 and center
    double delta_pos_p2p3_x_k = -pos_p2_x_k + pos_p3_x_k;
    double delta_pos_p2p3_y_k = -pos_p2_y_k + pos_p3_y_k;
    double delta_pos_p2p3_z_k = -pos_p2_z_k + pos_p3_z_k;
    double relprojection_p2p3_ki_val = (-delta_pos_p2p3_x_k*(pos_p2_x_k - pos_x_i) - delta_pos_p2p3_y_k*(pos_p2_y_k - pos_y_i) - delta_pos_p2p3_z_k*(pos_p2_z_k - pos_z_i))/(delta_pos_p2p3_x_k*delta_pos_p2p3_x_k + delta_pos_p2p3_y_k*delta_pos_p2p3_y_k + delta_pos_p2p3_z_k*delta_pos_p2p3_z_k);
    double projection_p2p3_x_ki = delta_pos_p2p3_x_k*relprojection_p2p3_ki_val + pos_p2_x_k;
    double projection_p2p3_y_ki = delta_pos_p2p3_y_k*relprojection_p2p3_ki_val + pos_p2_y_k;
    double projection_p2p3_z_ki = delta_pos_p2p3_z_k*relprojection_p2p3_ki_val + pos_p2_z_k;
    double dist_p2p3_ki_mag = sqrt((pos_x_i - projection_p2p3_x_ki)*(pos_x_i - projection_p2p3_x_ki) + (pos_y_i - projection_p2p3_y_ki)*(pos_y_i - projection_p2p3_y_ki) + (pos_z_i - projection_p2p3_z_ki)*(pos_z_i - projection_p2p3_z_ki));

    // calculate distance from edge P3 to P1 and center
    double delta_pos_p3p1_x_k = -pos_p3_x_k + pos_p1_x_k;
    double delta_pos_p3p1_y_k = -pos_p3_y_k + pos_p1_y_k;
    double delta_pos_p3p1_z_k = -pos_p3_z_k + pos_p1_z_k;
    double relprojection_p3p1_ki_val = (-delta_pos_p3p1_x_k*(pos_p3_x_k - pos_x_i) - delta_pos_p3p1_y_k*(pos_p3_y_k - pos_y_i) - delta_pos_p3p1_z_k*(pos_p3_z_k - pos_z_i))/(delta_pos_p3p1_x_k*delta_pos_p3p1_x_k + delta_pos_p3p1_y_k*delta_pos_p3p1_y_k + delta_pos_p3p1_z_k*delta_pos_p3p1_z_k);
    double projection_p3p1_x_ki = delta_pos_p3p1_x_k*relprojection_p3p1_ki_val + pos_p3_x_k;
    double projection_p3p1_y_ki = delta_pos_p3p1_y_k*relprojection_p3p1_ki_val + pos_p3_y_k;
    double projection_p3p1_z_ki = delta_pos_p3p1_z_k*relprojection_p3p1_ki_val + pos_p3_z_k;
    double dist_p3p1_ki_mag = sqrt((pos_x_i - projection_p3p1_x_ki)*(pos_x_i - projection_p3p1_x_ki) + (pos_y_i - projection_p3p1_y_ki)*(pos_y_i - projection_p3p1_y_ki) + (pos_z_i - projection_p3p1_z_ki)*(pos_z_i - projection_p3p1_z_ki));

    // calculate criteria for edge collision
    bool is_edge_p1p2_collision = dist_p1p2_ki_mag + TOLERANCE < radius_i;
    bool is_edge_p2p3_collision = dist_p2p3_ki_mag + TOLERANCE < radius_i;
    bool is_edge_p3p1_collision = dist_p3p1_ki_mag + TOLERANCE < radius_i;
    is_edge_collision = is_edge_p1p2_collision || is_edge_p2p3_collision || is_edge_p3p1_collision;

    // modify contact point if edge collision
    if(is_edge_p1p2_collision)
    {
        pos_contact_x = projection_p1p2_x_ki;
        pos_contact_y = projection_p1p2_y_ki;
        pos_contact_z = projection_p1p2_z_ki;
    }
    else if(is_edge_p2p3_collision)
    {
        pos_contact_x = projection_p2p3_x_ki;
        pos_contact_y = projection_p2p3_y_ki;
        pos_contact_z = projection_p2p3_z_ki;
    }
    else if(is_edge_p3p1_collision)
    {
        pos_contact_x = projection_p3p1_x_ki;
        pos_contact_y = projection_p3p1_y_ki;
        pos_contact_z = projection_p3p1_z_ki;
    }
    
}

template <class CollisionCheckSphereWallMesh>
void OutputSphereWallMeshForceMomentHertz<CollisionCheckSphereWallMesh>::check_vertex_collision(
    bool &is_vertex_collision,
    double &pos_contact_x, double &pos_contact_y, double &pos_contact_z,
    SpherePositionVelocityStruct &sphere_pvs,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    int indx_i, int indx_k
)
{
    /*

    Checks if sphere collides with vertex of mesh triangle.
    If so, calculates the contact point.

    */

    // get particle and wall type
    int type_i = sphere_pvs.type_vec[indx_i];
    int type_k = wallmesh_pvs.type_vec[indx_k];

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

    // get particle radius
    double radius_i = radius_vec[type_i];

    // calculate distance from P1 to center
    double delta_pos_p1_x_ik = -pos_x_i + pos_p1_x_k;
    double delta_pos_p1_y_ik = -pos_y_i + pos_p1_y_k;
    double delta_pos_p1_z_ik = -pos_z_i + pos_p1_z_k;
    double delta_pos_p1_ik_mag = sqrt(delta_pos_p1_x_ik*delta_pos_p1_x_ik + delta_pos_p1_y_ik*delta_pos_p1_y_ik + delta_pos_p1_z_ik*delta_pos_p1_z_ik);

    // calculate distance from P2 to center
    double delta_pos_p2_x_ik = -pos_x_i + pos_p2_x_k;
    double delta_pos_p2_y_ik = -pos_y_i + pos_p2_y_k;
    double delta_pos_p2_z_ik = -pos_z_i + pos_p2_z_k;
    double delta_pos_p2_ik_mag = sqrt(delta_pos_p2_x_ik*delta_pos_p2_x_ik + delta_pos_p2_y_ik*delta_pos_p2_y_ik + delta_pos_p2_z_ik*delta_pos_p2_z_ik);

    // calculate distance from P3 to center
    double delta_pos_p3_x_ik = -pos_x_i + pos_p3_x_k;
    double delta_pos_p3_y_ik = -pos_y_i + pos_p3_y_k;
    double delta_pos_p3_z_ik = -pos_z_i + pos_p3_z_k;
    double delta_pos_p3_ik_mag = sqrt(delta_pos_p3_x_ik*delta_pos_p3_x_ik + delta_pos_p3_y_ik*delta_pos_p3_y_ik + delta_pos_p3_z_ik*delta_pos_p3_z_ik);

    // calculate criteria for vertex collision
    bool is_vertex_p1_collision = delta_pos_p1_ik_mag < radius_i;
    bool is_vertex_p2_collision = delta_pos_p2_ik_mag < radius_i;
    bool is_vertex_p3_collision = delta_pos_p3_ik_mag < radius_i;
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

template <class CollisionCheckSphereWallMesh>
void OutputSphereWallMeshForceMomentHertz<CollisionCheckSphereWallMesh>::calculate_velocity_contact(
    double &vel_contact_x, double &vel_contact_y, double &vel_contact_z,
    WallMeshPositionVelocityStruct &wallmesh_pvs,
    double pos_contact_x, double pos_contact_y, double pos_contact_z
)
{
    /*

    Calculates the velocity of a mesh triangle at a given contact point.

    */

    // get wall velocities
    double vel_translate_x = wallmesh_pvs.velocity_translate_x;
    double vel_translate_y = wallmesh_pvs.velocity_translate_y;
    double vel_translate_z = wallmesh_pvs.velocity_translate_z;
    double angvel_rotate = wallmesh_pvs.angularvelocity_rotate;

    // get rotation axis points
    double pos_rotateaxis_p1_x = wallmesh_pvs.position_rotateaxis_p1_x;
    double pos_rotateaxis_p1_y = wallmesh_pvs.position_rotateaxis_p1_y;
    double pos_rotateaxis_p1_z = wallmesh_pvs.position_rotateaxis_p1_z;
    double pos_rotateaxis_p2_x = wallmesh_pvs.position_rotateaxis_p2_x;
    double pos_rotateaxis_p2_y = wallmesh_pvs.position_rotateaxis_p2_y;
    double pos_rotateaxis_p2_z = wallmesh_pvs.position_rotateaxis_p2_z;

    // axis of rotation is specified using two points: axis P1 and axis P2
    // calculate vector from axis P1 to axis P2 (labeled A1 and A2)
    double rotateaxis_x = -pos_rotateaxis_p1_x + pos_rotateaxis_p2_x;
    double rotateaxis_y = -pos_rotateaxis_p1_y + pos_rotateaxis_p2_y;
    double rotateaxis_z = -pos_rotateaxis_p1_z + pos_rotateaxis_p2_z;

    // normalize vector along axis of rotation
    double helpvar_01 = 1./sqrt(rotateaxis_x*rotateaxis_x + rotateaxis_y*rotateaxis_y + rotateaxis_z*rotateaxis_z);
    double unit_rotate_axis_x = rotateaxis_x*helpvar_01;
    double unit_rotate_axis_y = rotateaxis_y*helpvar_01;
    double unit_rotate_axis_z = rotateaxis_z*helpvar_01;
    if (abs(rotateaxis_x) < TOLERANCE && abs(rotateaxis_y) < TOLERANCE && abs(rotateaxis_z) < TOLERANCE)
    {
        unit_rotate_axis_x = 0.;
        unit_rotate_axis_y = 0.;
        unit_rotate_axis_z = 0.;
    }

    // calculate vector from axis P1 to contact point
    double delta_pos_p1_contact_x = -pos_rotateaxis_p1_x + pos_contact_x;
    double delta_pos_p1_contact_y = -pos_rotateaxis_p1_y + pos_contact_y;
    double delta_pos_p1_contact_z = -pos_rotateaxis_p1_z + pos_contact_z;

    // calculate distance from axis P1 to closest point
    // closest point - point along axis closest to contact point
    double delta_pos_p1_close_val = delta_pos_p1_contact_x*unit_rotate_axis_x + delta_pos_p1_contact_y*unit_rotate_axis_y + delta_pos_p1_contact_z*unit_rotate_axis_z;

    // calculate vector from axis P1 to closest point
    double delta_pos_p1_close_x = delta_pos_p1_close_val*unit_rotate_axis_x;
    double delta_pos_p1_close_y = delta_pos_p1_close_val*unit_rotate_axis_y;
    double delta_pos_p1_close_z = delta_pos_p1_close_val*unit_rotate_axis_z;

    // calculate vector from closest point to contact point
    double delta_close_contact_x = -delta_pos_p1_close_x + delta_pos_p1_contact_x;
    double delta_close_contact_y = -delta_pos_p1_close_y + delta_pos_p1_contact_y;
    double delta_close_contact_z = -delta_pos_p1_close_z + delta_pos_p1_contact_z;

    // calculate rotational velocity at contact point
    double vel_rotate_x = angvel_rotate*(-delta_close_contact_y*unit_rotate_axis_z + delta_close_contact_z*unit_rotate_axis_y);
    double vel_rotate_y = angvel_rotate*( delta_close_contact_x*unit_rotate_axis_z - delta_close_contact_z*unit_rotate_axis_x);
    double vel_rotate_z = angvel_rotate*(-delta_close_contact_x*unit_rotate_axis_y + delta_close_contact_y*unit_rotate_axis_x);

    // calculate velocity at contact point
    vel_contact_x = vel_translate_x + vel_rotate_x;
    vel_contact_y = vel_translate_y + vel_rotate_y;
    vel_contact_z = vel_translate_z + vel_rotate_z;

}

#endif
