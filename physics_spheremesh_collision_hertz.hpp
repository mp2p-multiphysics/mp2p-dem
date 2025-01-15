#ifndef PHYSICS_SPHEREMESH_COLLISION_HERTZ
#define PHYSICS_SPHEREMESH_COLLISION_HERTZ
#include <set>
#include "collider_spheremesh_base.hpp"
#include "parameter_1d.hpp"
#include "parameter_2d.hpp"
#include "physics_base.hpp"

namespace DEM
{

class PhysicsSphereMeshCollisionHertz : public PhysicsBase
{

    public:
    
    // collider
    ColliderSphereMeshBase* collider_ptr;
    CollisionSphereMeshVector collision_vec;

    // sets with edge or vertex collisions
    CollisionSphereMeshVector collision_face_vec;
    std::set<std::tuple<SpherePIDPair, MeshPIDPair, int, int>> collision_edge_set;  // int, int -> pid of edges
    std::set<std::tuple<SpherePIDPair, MeshPIDPair, int>> collision_vertex_set;  // int -> pid of vertex

    // material parameters
    Parameter1D* density_ptr;
    Parameter2D* spring_normal_ptr;
    Parameter2D* spring_tangent_ptr;
    Parameter2D* damping_normal_ptr;
    Parameter2D* damping_tangent_ptr;
    Parameter2D* friction_sliding_ptr;
    Parameter2D* friction_rolling_ptr;

    // tangential overlap matrices
    CollisionSphereMeshMatrix overlap_tangent_mat;
    CollisionSphereMeshMatrix ddt_overlap_tangent_mat;

    // functions used in timestepping
    void compute_force(int ts);
    void compute_position_velocity(int ts) {};
    void compute_insert_delete(int ts) {};

    // default constructor
    PhysicsSphereMeshCollisionHertz() {};

    // constructor
    PhysicsSphereMeshCollisionHertz
    (
        ColliderSphereMeshBase &collider_in, Parameter1D &density_in,
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
    void classify_collide_pair(CollisionSphereMeshPair collision_pair);
    void compute_force_face_pair(CollisionSphereMeshPair collision_face_pair);
    void compute_force_edge_pair(std::tuple<SpherePIDPair, MeshPIDPair, int, int> collision_edge_pair);
    void compute_force_vertex_pair(std::tuple<SpherePIDPair, MeshPIDPair, int> collision_vertex_pair);
    void compute_force_general(ParticleSphere* sphere_i_ptr, WallMesh* mesh_j_ptr, int pid_i, int fid_j, EigenVector3D contact_ij);

};

void PhysicsSphereMeshCollisionHertz::compute_force(int ts)
{

    // update collider list
    collider_ptr->compute_collide_pair(collision_vec, ts);

    //　classify into face, edge, or vertex
    for (auto collision_pair : collision_vec)
    {
        classify_collide_pair(collision_pair);
    }

    // calculate forces
    for (auto collision_face_pair : collision_face_vec)
    {
        compute_force_face_pair(collision_face_pair);
    }
    for (auto collision_edge_pair : collision_edge_set)
    {
        compute_force_edge_pair(collision_edge_pair);
    }
    for (auto collision_vertex_pair : collision_vertex_set)
    {
        compute_force_vertex_pair(collision_vertex_pair);
    }

}

void PhysicsSphereMeshCollisionHertz::classify_collide_pair(CollisionSphereMeshPair collision_pair)
{

    // get sphere-ID and face-ID pairs
    SpherePIDPair sphere_pid_i = collision_pair.first;
    MeshPIDPair mesh_fid_j = collision_pair.second;
    
    // sphere i
    ParticleSphere* sphere_i_ptr = sphere_pid_i.first;
    int pid_i = sphere_pid_i.second;

    // mesh j
    WallMesh* mesh_j_ptr = mesh_fid_j.first;
    int fid_j = mesh_fid_j.second;

    // get ID for array indexing
    int tid_i = sphere_i_ptr->pid_to_tid_map[pid_i];
    int pid_p1_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][0];
    int pid_p2_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][1];
    int pid_p3_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][2];

    // get particle position and radius
    EigenVector3D pos_i = sphere_i_ptr->position_vec[tid_i];
    double radius_i = sphere_i_ptr->radius_vec[tid_i];

    // get wall points
    EigenVector3D pos_p1_j = mesh_j_ptr->position_vec[pid_p1_j];
    EigenVector3D pos_p2_j = mesh_j_ptr->position_vec[pid_p2_j];
    EigenVector3D pos_p3_j = mesh_j_ptr->position_vec[pid_p3_j];

    // calculate vector facing the "front" of the triangle
    EigenVector3D front_j = (pos_p2_j - pos_p1_j).cross(pos_p3_j - pos_p1_j);

    // calculate front facing normal vectors
    EigenVector3D norm_j = front_j/front_j.norm();

    // calculate front facing wall to particle distance
    double dist_ij_val = norm_j.dot(pos_i - pos_p1_j);

    // determine whether to use front or back facing normal and distance
    // particle faces the front if the distance from the front is positive
    if (dist_ij_val < 0)
    {
        norm_j *= -1.;
        dist_ij_val *= -1.;
    }

    // skip particle is too far for collision
    if (dist_ij_val < radius_i)
    {
        overlap_tangent_mat.prune(collision_pair);
        ddt_overlap_tangent_mat.prune(collision_pair);
        return;
    }

    // check for sphere-face collision

    // calculate (preliminary) contact point
    EigenVector3D contact_ij = -dist_ij_val*norm_j + pos_i;

    // calculate test vectors
    EigenVector3D face_test_p1p2_vec_ij = (pos_p1_j - contact_ij).cross(pos_p2_j - contact_ij);
    EigenVector3D face_test_p2p3_vec_ij = (pos_p2_j - contact_ij).cross(pos_p3_j - contact_ij);
    EigenVector3D face_test_p3p1_vec_ij = (pos_p3_j - contact_ij).cross(pos_p1_j - contact_ij);

    // calculate criteria for face collision
    double face_test_criteria_1 = face_test_p1p2_vec_ij.dot(face_test_p2p3_vec_ij);
    double face_test_criteria_2 = face_test_p1p2_vec_ij.dot(face_test_p3p1_vec_ij);
    bool is_face_collision = face_test_criteria_1 > 1e-8 && face_test_criteria_2 > 1e-8;

    // if not face collision, then check for edge collision
    if (is_face_collision)
    {
        collision_face_vec.push_back(collision_pair);
        return;
    }
    else
    {
        
        // calculate distance from center to p1-p2 edge
        EigenVector3D delta_pos_p1p2_j = pos_p2_j - pos_p1_j;
        double relprojection_p1p2_ji_val = (pos_i - pos_p1_j).dot(delta_pos_p1p2_j) / delta_pos_p1p2_j.dot(delta_pos_p1p2_j);
        EigenVector3D projection_p1p2_ji = delta_pos_p1p2_j*relprojection_p1p2_ji_val + pos_p1_j;
        double dist_p1p2_ji_mag = (pos_i - projection_p1p2_ji).norm();
        
        // calculate distance from center to p2-p3 edge
        EigenVector3D delta_pos_p2p3_j = pos_p3_j - pos_p2_j;
        double relprojection_p2p3_ji_val = (pos_i - pos_p2_j).dot(delta_pos_p2p3_j) / delta_pos_p2p3_j.dot(delta_pos_p2p3_j);
        EigenVector3D projection_p2p3_ji = delta_pos_p2p3_j*relprojection_p2p3_ji_val + pos_p2_j;
        double dist_p2p3_ji_mag = (pos_i - projection_p2p3_ji).norm();

        // calculate distance from center to p3-p1 edge
        EigenVector3D delta_pos_p3p1_j = pos_p1_j - pos_p3_j;
        double relprojection_p3p1_ji_val = (pos_i - pos_p3_j).dot(delta_pos_p3p1_j) / delta_pos_p3p1_j.dot(delta_pos_p3p1_j);
        EigenVector3D projection_p3p1_ji = delta_pos_p3p1_j*relprojection_p3p1_ji_val + pos_p3_j;
        double dist_p3p1_ji_mag = (pos_i - projection_p3p1_ji).norm();        

        // calculate criteria for edge collision
        bool is_p1p2_collision = ((dist_p1p2_ji_mag + 1e-8) < radius_i);
        bool is_p2p3_collision = ((dist_p2p3_ji_mag + 1e-8) < radius_i);
        bool is_p3p1_collision = ((dist_p3p1_ji_mag + 1e-8) < radius_i);
        bool is_edge_collision = is_p1p2_collision || is_p2p3_collision || is_p3p1_collision;

        // store edge if collision occurs
        if (is_p1p2_collision)
        {
            int pid_pa = pid_p1_j;
            int pid_pb = pid_p2_j;
            if (pid_pa < pid_pb)
            {
                pid_pa = pid_p2_j;
                pid_pb = pid_p1_j;
            }
            collision_edge_set.insert({sphere_pid_i, mesh_fid_j, pid_pa, pid_pb});
        }
        if (is_p2p3_collision)
        {
            int pid_pa = pid_p2_j;
            int pid_pb = pid_p3_j;
            if (pid_pa < pid_pb)
            {
                pid_pa = pid_p3_j;
                pid_pb = pid_p2_j;
            }
            collision_edge_set.insert({sphere_pid_i, mesh_fid_j, pid_pa, pid_pb});
        }
        if (is_p3p1_collision)
        {
            int pid_pa = pid_p3_j;
            int pid_pb = pid_p1_j;
            if (pid_pa < pid_pb)
            {
                pid_pa = pid_p1_j;
                pid_pb = pid_p3_j;
            }
            collision_edge_set.insert({sphere_pid_i, mesh_fid_j, pid_pa, pid_pb});
        }

        // if not face collision, then check for vertex collision
        if (is_edge_collision)
        {
            return;
        }
        else
        {

            // calculate distance from points
            double dist_p1_ij_mag = (pos_p1_j - pos_i).norm();
            double dist_p2_ij_mag = (pos_p2_j - pos_i).norm();
            double dist_p3_ij_mag = (pos_p3_j - pos_i).norm();

            // calculate criteria for vertex collision
            bool is_vertex_p1_collision = dist_p1_ij_mag < radius_i;
            bool is_vertex_p2_collision = dist_p2_ij_mag < radius_i;
            bool is_vertex_p3_collision = dist_p3_ij_mag < radius_i;
            bool is_vertex_collision = is_vertex_p1_collision || is_vertex_p2_collision || is_vertex_p3_collision;

            // store vertex if collision occurs
            if(is_vertex_p1_collision)
            {
                collision_vertex_set.insert({sphere_pid_i, mesh_fid_j, pid_p1_j});
            }
            if(is_vertex_p2_collision)
            {
                collision_vertex_set.insert({sphere_pid_i, mesh_fid_j, pid_p2_j});
            }
            if(is_vertex_p3_collision)
            {
                collision_vertex_set.insert({sphere_pid_i, mesh_fid_j, pid_p3_j});
            }

            // skip if not vertex collision
            if (is_vertex_collision)
            {
                return;
            }
            else
            {
                overlap_tangent_mat.prune(collision_pair);
                ddt_overlap_tangent_mat.prune(collision_pair);
                return;
            }

        }

    }

}

void PhysicsSphereMeshCollisionHertz::compute_force_face_pair(CollisionSphereMeshPair collision_face_pair)
{

    // get sphere-ID and face-ID pairs
    SpherePIDPair sphere_pid_i = collision_face_pair.first;
    MeshPIDPair mesh_fid_j = collision_face_pair.second;
    
    // sphere i
    ParticleSphere* sphere_i_ptr = sphere_pid_i.first;
    int pid_i = sphere_pid_i.second;

    // mesh j
    WallMesh* mesh_j_ptr = mesh_fid_j.first;
    int fid_j = mesh_fid_j.second;

    // get ID for array indexing
    int tid_i = sphere_i_ptr->pid_to_tid_map[pid_i];
    int pid_p1_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][0];
    int pid_p2_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][1];
    int pid_p3_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][2];

    // get particle position and radius
    EigenVector3D pos_i = sphere_i_ptr->position_vec[tid_i];
    double radius_i = sphere_i_ptr->radius_vec[tid_i];

    // get wall points
    EigenVector3D pos_p1_j = mesh_j_ptr->position_vec[pid_p1_j];
    EigenVector3D pos_p2_j = mesh_j_ptr->position_vec[pid_p2_j];
    EigenVector3D pos_p3_j = mesh_j_ptr->position_vec[pid_p3_j];

    // calculate vector facing the "front" of the triangle
    EigenVector3D front_j = (pos_p2_j - pos_p1_j).cross(pos_p3_j - pos_p1_j);

    // calculate front facing normal vectors
    EigenVector3D norm_j = front_j/front_j.norm();

    // calculate front facing wall to particle distance
    double dist_ij_val = norm_j.dot(pos_i - pos_p1_j);

    // determine whether to use front or back facing normal and distance
    // particle faces the front if the distance from the front is positive
    if (dist_ij_val < 0)
    {
        norm_j *= -1.;
        dist_ij_val *= -1.;
    }

    // calculate contact point
    EigenVector3D contact_ij = -dist_ij_val*norm_j + pos_i;

    // calculate force
    compute_force_general(sphere_i_ptr, mesh_j_ptr, pid_i, fid_j, contact_ij);

}

void PhysicsSphereMeshCollisionHertz::compute_force_edge_pair(std::tuple<SpherePIDPair, MeshPIDPair, int, int> collision_edge_pair)
{

    // get sphere-ID and face-ID pairs
    SpherePIDPair sphere_pid_i = std::get<0>(collision_edge_pair);
    MeshPIDPair mesh_fid_j = std::get<1>(collision_edge_pair);
    
    // sphere i
    ParticleSphere* sphere_i_ptr = sphere_pid_i.first;
    int pid_i = sphere_pid_i.second;

    // mesh j
    WallMesh* mesh_j_ptr = mesh_fid_j.first;
    int fid_j = mesh_fid_j.second;

    // get ID for array indexing
    int tid_i = sphere_i_ptr->pid_to_tid_map[pid_i];
    int pid_pa_j = std::get<2>(collision_edge_pair);
    int pid_pb_j = std::get<3>(collision_edge_pair);

    // get particle position and radius
    EigenVector3D pos_i = sphere_i_ptr->position_vec[tid_i];
    double radius_i = sphere_i_ptr->radius_vec[tid_i];

    // get wall points
    EigenVector3D pos_pa_j = mesh_j_ptr->position_vec[pid_pa_j];
    EigenVector3D pos_pb_j = mesh_j_ptr->position_vec[pid_pb_j];

    // calculate contact point to edge
    EigenVector3D delta_pos_papb_j = pos_pb_j - pos_pa_j;
    double relprojection_papb_ij_val = (pos_i - pos_pa_j).dot(delta_pos_papb_j) / delta_pos_papb_j.dot(delta_pos_papb_j);
    EigenVector3D contact_ij = delta_pos_papb_j*relprojection_papb_ij_val + pos_pa_j;

    // calculate force
    compute_force_general(sphere_i_ptr, mesh_j_ptr, pid_i, fid_j, contact_ij);

}

void PhysicsSphereMeshCollisionHertz::compute_force_vertex_pair(std::tuple<SpherePIDPair, MeshPIDPair, int> collision_vertex_pair)
{

    // get sphere-ID and face-ID pairs
    SpherePIDPair sphere_pid_i = std::get<0>(collision_vertex_pair);
    MeshPIDPair mesh_fid_j = std::get<1>(collision_vertex_pair);
    
    // sphere i
    ParticleSphere* sphere_i_ptr = sphere_pid_i.first;
    int pid_i = sphere_pid_i.second;

    // mesh j
    WallMesh* mesh_j_ptr = mesh_fid_j.first;
    int fid_j = mesh_fid_j.second;

    // get ID for array indexing
    int pid_j = std::get<2>(collision_vertex_pair);

    // get wall points
    // also the contact point
    EigenVector3D contact_ij = mesh_j_ptr->position_vec[pid_j];

    // calculate force
    compute_force_general(sphere_i_ptr, mesh_j_ptr, pid_i, fid_j, contact_ij);

}

void PhysicsSphereMeshCollisionHertz::compute_force_general(ParticleSphere* sphere_i_ptr, WallMesh* mesh_j_ptr, int pid_i, int fid_j, EigenVector3D contact_ij)
{

    // get ID for array indexing
    int tid_i = sphere_i_ptr->pid_to_tid_map[pid_i];
    int pid_p1_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][0];
    int pid_p2_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][1];
    int pid_p3_j = mesh_j_ptr->fid_lid_to_pid_vec[fid_j][2];

    // get particle position and radius
    EigenVector3D pos_i = sphere_i_ptr->position_vec[tid_i];
    double radius_i = sphere_i_ptr->radius_vec[tid_i];

    // calculate normal vector
    EigenVector3D delta_pos_ij = pos_i - contact_ij;
    double delta_pos_ij_mag = delta_pos_ij.norm();
    EigenVector3D normal_ij = delta_pos_ij/delta_pos_ij_mag;

    // get particle velocities
    EigenVector3D vel_i = sphere_i_ptr->velocity_vec[tid_i];
    EigenVector3D angvel_i = sphere_i_ptr->angularvelocity_vec[tid_i];

    // get contact point velocity

    // get wall velocities
    EigenVector3D vel_trn_j = mesh_j_ptr->velocity_vec;
    double angvel_rot_j = mesh_j_ptr->angularvelocity_vec;
    EigenVector3D pos_rotaxis_p1_j = mesh_j_ptr->rotateaxis_p1_vec;
    EigenVector3D pos_rotaxis_p2_j = mesh_j_ptr->rotateaxis_p2_vec;

    // get axis of rotation
    EigenVector3D delta_pos_rotaxis_p1p2_j = pos_rotaxis_p2_j - pos_rotaxis_p1_j;
    double delta_pos_rotaxis_p1p2_j_mag = delta_pos_rotaxis_p1p2_j.norm();
    EigenVector3D unit_rotaxis_p1p2_j = delta_pos_rotaxis_p1p2_j/delta_pos_rotaxis_p1p2_j_mag;
    if (delta_pos_rotaxis_p1p2_j_mag < 1e-8)
    {
        unit_rotaxis_p1p2_j = {0., 0., 0.};
    }

    // calculate rotational velocity at contact point
    EigenVector3D delta_pos_p1_contact = contact_ij - pos_rotaxis_p1_j;  // p1 to contact point
    double delta_pos_p1_close_val = delta_pos_p1_contact.dot(unit_rotaxis_p1p2_j);  // p1 to "close" point (projection of contact point on rotation axis)
    EigenVector3D delta_pos_p1_close = delta_pos_p1_close_val*unit_rotaxis_p1p2_j;
    EigenVector3D delta_close_contact = delta_pos_p1_contact - delta_pos_p1_close;
    EigenVector3D vel_rot_j = angvel_rot_j*unit_rotaxis_p1p2_j.cross(delta_close_contact);

    // calculate velocity at contact point
    EigenVector3D vel_j = vel_trn_j + vel_rot_j;

    // get material type
    int material_i = sphere_i_ptr->material_vec[tid_i];
    int material_j = mesh_j_ptr->material;

    // get collision properties
    double spring_normal = spring_normal_ptr->get_value(material_i, material_j);
    double spring_tangent = spring_tangent_ptr->get_value(material_i, material_j);
    double damping_normal = damping_normal_ptr->get_value(material_i, material_j);
    double damping_tangent = damping_tangent_ptr->get_value(material_i, material_j);
    double friction_sliding = friction_sliding_ptr->get_value(material_i, material_j);
    double friction_rolling = friction_rolling_ptr->get_value(material_i, material_j);

    // get tangential overlap
    CollisionSphereMeshPair collision_pair = {{sphere_i_ptr, pid_i}, {mesh_j_ptr, fid_j}};
    double overlap_tangent_ij_val = overlap_tangent_mat.get_value(collision_pair);

    // calculate relative velocity vector
    EigenVector3D relvel_ij = vel_i - vel_j + (radius_i*angvel_i).cross(normal_ij);

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

    // calculate normal overlap
    double overlap_normal_ij_val = radius_i - delta_pos_ij_mag;

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
    // sphere_j_ptr->force_vec[tid_j] += -force_collision_ij;

    // calculate collision moment on i
    // same direction for i and j since both collision force and normal vector switch signs
    EigenVector3D moment_collision_ij = radius_i*normal_ij.cross(force_collision_tangent_ij);
    // EigenVector3D moment_collision_ji = radius_j*normal_ij.cross(force_collision_tangent_ij);

    // calculate relative angular velocities of i and j
    // EigenVector3D relangvel_ij = angvel_i - angvel_j;
    EigenVector3D relangvel_ij = angvel_i;
    double relangvel_ij_mag = relangvel_ij.norm();

    // calculate unit vector pointing to relative angular velocity
    // unit vector is zero if difference between angular velocities is zero
    EigenVector3D axis_ij = relangvel_ij/relangvel_ij_mag;
    if (relangvel_ij_mag < 1e-8)
    {
        axis_ij = {0., 0., 0.};
    }

    // calculate effective radius
    // double radius_effective = radius_i*radius_j/(radius_i + radius_j);
    double radius_effective = radius_i;

    // calculate magnitude of fricion moment
    double moment_friction_ij_val = -force_collision_normal_ij_mag*friction_rolling*radius_effective;

    // calculate friction moment on i
    // axis switches signs for j
    EigenVector3D moment_friction_ij = moment_friction_ij_val*axis_ij;
    // EigenVector3D moment_friction_ji = -moment_friction_ij;

    // add moments on i and j
    sphere_i_ptr->moment_vec[tid_i] += moment_collision_ij + moment_friction_ij;
    // sphere_j_ptr->moment_vec[tid_j] += moment_collision_ji + moment_friction_ji;

}

}

#endif
