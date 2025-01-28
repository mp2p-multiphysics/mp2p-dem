#ifndef FORCEMOMENT_SPHEREMESH_HERTZ
#define FORCEMOMENT_SPHEREMESH_HERTZ
#include <algorithm>
#include "collider_spheremesh_base.hpp"
#include "container_typedef.hpp"
#include "forcemoment_base.hpp"
#include "group_sphere.hpp"
#include "group_mesh.hpp"
#include "parameter_binary.hpp"

namespace DEM
{

class ForceMomentSphereMeshHertz : public ForceMomentBase
{
    /*

    Computes collision forces and moments between spheres and meshes via the Hertz model.

    Variables
    =========
    spheregroup_in : SphereGroup
        Spheres where forces and moments are applied.
    meshgroup_in : MeshGroup
        Meshes where forces and moments are applied.
    collider_in : ColliderSphereMeshBase
        Broad phase collision checker.
    spring_normal_in : ParameterBinary
        Spring constant in the normal direction.
    spring_tangent_in : ParameterBinary
        Spring constant in the tangential direction.
    damping_normal_in : ParameterBinary
        Damping coefficient in the normal direction.
    damping_tangent_in : ParameterBinary
        Damping coefficient in the tangential direction.
    friction_sliding_in : ParameterBinary
        Sliding friction coefficient.
    friction_rolling_in : ParameterBinary
        Rolling friction coefficient.

    Functions
    =========
    get_group_ptr_vec : vector<BaseGroup*>
        Returns pointers to group objects affected by this object.
    update : void
        Updates this object.

    */

    public:

    // sphere group
    ColliderSphereMeshBase* collider_ptr;
    SphereGroup* spheregroup_ptr;
    MeshGroup* meshgroup_ptr;

    // parameters
    ParameterBinary* spring_normal_ptr;
    ParameterBinary* spring_tangent_ptr;
    ParameterBinary* damping_normal_ptr;
    ParameterBinary* damping_tangent_ptr;
    ParameterBinary* friction_sliding_ptr;
    ParameterBinary* friction_rolling_ptr;

    // tangential overlap list
    std::map<std::pair<int, int>, double> overlap_tangent_map;

    // unique edge pair and vertex vectors
    std::vector<std::pair<int, int>> edge_pair_vec;
    std::vector<int> vertex_vec;

    // functions
    std::vector<BaseGroup*> get_group_ptr_vec() {return {spheregroup_ptr, meshgroup_ptr};};
    void update(int ts, double dt);

    // default constructor
    ForceMomentSphereMeshHertz() {}

    // constructor
    ForceMomentSphereMeshHertz
    (
        SphereGroup &spheregroup_in, MeshGroup &meshgroup_in, ColliderSphereMeshBase &collider_in,
        ParameterBinary &spring_normal_in, ParameterBinary &spring_tangent_in,
        ParameterBinary &damping_normal_in, ParameterBinary &damping_tangent_in,
        ParameterBinary &friction_sliding_in, ParameterBinary &friction_rolling_in
    )
    {

        // store sphere group
        collider_ptr = &collider_in;
        spheregroup_ptr = &spheregroup_in;
        meshgroup_ptr = &meshgroup_in;

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
    double get_overlap_tangent_value(std::pair<int, int> collision_pair);

};

void ForceMomentSphereMeshHertz::update(int ts, double dt)
{
    /*

    Updates this object.

    Arguments
    =========
    ts : int
        Timestep number.
    dt : double
        Duration of timestep.

    Returns
    =======
    (none)

    */

    // update vector of collision pairs
    collider_ptr->update_collision_vec(ts);
    
    // reset vectors of unique edge pairs and vertices
    edge_pair_vec.clear();
    vertex_vec.clear();

    // iterate through each pair
    for (auto indx_pair : collider_ptr->get_collision_vec())
    {
        compute_force_pair(indx_pair.first, indx_pair.second, dt);
    }

}

double ForceMomentSphereMeshHertz::get_overlap_tangent_value(std::pair<int, int> collision_pair)
{

    // find key in map
    auto iter = overlap_tangent_map.find(collision_pair);

    // return value if found
    if (iter != overlap_tangent_map.end()) {
        return iter->second;
    }

    // return zero otherwise
    // also append zero to map
    overlap_tangent_map[collision_pair] = 0.;
    return 0.;

}

void ForceMomentSphereMeshHertz::compute_force_pair(int indx_i, int indx_j, double dt)
{

    // update collider
    // compute assuming face collision
    // if not face collision -> record particle and edge; compute assuming edge collision and first time
    // if not edge collision -> record particle and vertex; compute assuming vertex collision and first time

    // get spheres and meshes
    Sphere sphere_i = spheregroup_ptr->sphere_vec[indx_i];
    Mesh mesh_j = meshgroup_ptr->mesh_vec[indx_j];

    // get particle geometry
    EigenVector3D pos_i = sphere_i.position;
    double radius_i = sphere_i.radius;

    // get wall geometry
    EigenVector3D pos_p0_j = mesh_j.position_p0;
    EigenVector3D pos_p1_j = mesh_j.position_p1;
    EigenVector3D pos_p2_j = mesh_j.position_p2;

    // get particle group ID pair
    std::pair<int, int> collision_pair = {sphere_i.gid, indx_j};

    // calculate normal vector to face
    EigenVector3D front_j = (pos_p1_j - pos_p0_j).cross(pos_p2_j - pos_p0_j);
    EigenVector3D norm_j = front_j/front_j.norm();

    // calculate wall to particle distance
    double dist_ij_val = norm_j.dot(pos_i - pos_p1_j);

    // determine whether to use front or back facing normal and distance
    // particle faces the front if the distance from the front is positive
    if (dist_ij_val < 0)
    {
        norm_j *= -1.;
        dist_ij_val *= -1.;
    }

    // skip if particle is too far for collision
    if (dist_ij_val > radius_i)
    {
        overlap_tangent_map.erase(collision_pair);  // reset tangential overlap
        return;
    }

    // compute contact point between sphere and mesh
    // also determine if collision is on face, edge, or vertex

    // assume sphere-face collision

    // calculate contact point
    EigenVector3D contact_ij = -dist_ij_val*norm_j + pos_i;

    // calculate test vectors
    EigenVector3D face_test_p0p1_vec_ij = (pos_p0_j - contact_ij).cross(pos_p1_j - contact_ij);
    EigenVector3D face_test_p1p2_vec_ij = (pos_p1_j - contact_ij).cross(pos_p2_j - contact_ij);
    EigenVector3D face_test_p2p0_vec_ij = (pos_p2_j - contact_ij).cross(pos_p0_j - contact_ij);

    // calculate criteria for face collision
    double face_test_criteria_1 = face_test_p0p1_vec_ij.dot(face_test_p1p2_vec_ij);
    double face_test_criteria_2 = face_test_p0p1_vec_ij.dot(face_test_p2p0_vec_ij);
    bool is_face_collision = face_test_criteria_1 > 1e-8 && face_test_criteria_2 > 1e-8;

    // if not face collision, then check for edge collision
    if (!is_face_collision)
    {

        // calculate distance from center to p0-p1 edge
        EigenVector3D delta_pos_p0p1_j = pos_p1_j - pos_p0_j;
        double relprojection_p0p1_ji_val = (pos_i - pos_p0_j).dot(delta_pos_p0p1_j) / delta_pos_p0p1_j.dot(delta_pos_p0p1_j);
        EigenVector3D projection_p0p1_ji = delta_pos_p0p1_j*relprojection_p0p1_ji_val + pos_p0_j;
        double dist_p0p1_ji_mag = (pos_i - projection_p0p1_ji).norm();

        // calculate criteria for p0-p1 edge collision
        bool is_p0p1_collision = ((dist_p0p1_ji_mag + 1e-8) < radius_i);

        // determine contact point if p0-p1 edge collision
        if (is_p0p1_collision)
        {
            
            // skip if edge has been tested before
            // avoid double counting of forces
            std::pair<int, int> edge_pair = {mesh_j.pid_p0, mesh_j.pid_p1};
            if (mesh_j.pid_p0 > mesh_j.pid_p1) {edge_pair = {mesh_j.pid_p1, mesh_j.pid_p0};}  // let first < second for consistency
            auto iter = std::find(edge_pair_vec.begin(), edge_pair_vec.end(), edge_pair);
            if (iter != edge_pair_vec.end()) {return;}  // skip if edge is found
            
            // record edge if force not yet computed
            edge_pair_vec.push_back(edge_pair);

            // calculate contact point to edge
            EigenVector3D delta_pos_p0p1_j = pos_p1_j - pos_p0_j;
            double relprojection_p0p1_ij_val = (pos_i - pos_p0_j).dot(delta_pos_p0p1_j) / delta_pos_p0p1_j.dot(delta_pos_p0p1_j);
            contact_ij = delta_pos_p0p1_j*relprojection_p0p1_ij_val + pos_p0_j;

        }

        // calculate distance from center to p1-p2 edge
        EigenVector3D delta_pos_p1p2_j = pos_p2_j - pos_p1_j;
        double relprojection_p1p2_ji_val = (pos_i - pos_p1_j).dot(delta_pos_p1p2_j) / delta_pos_p1p2_j.dot(delta_pos_p1p2_j);
        EigenVector3D projection_p1p2_ji = delta_pos_p1p2_j*relprojection_p1p2_ji_val + pos_p1_j;
        double dist_p1p2_ji_mag = (pos_i - projection_p1p2_ji).norm();

        // calculate criteria for p1-p2 edge collision
        bool is_p1p2_collision = ((dist_p1p2_ji_mag + 1e-8) < radius_i);

        // determine contact point if p1-p2 edge collision
        if (is_p1p2_collision)
        {
            
            // skip if edge has been tested before
            // avoid double counting of forces
            std::pair<int, int> edge_pair = {mesh_j.pid_p1, mesh_j.pid_p2};
            if (mesh_j.pid_p1 > mesh_j.pid_p2) {edge_pair = {mesh_j.pid_p2, mesh_j.pid_p1};}  // let first < second for consistency
            auto iter = std::find(edge_pair_vec.begin(), edge_pair_vec.end(), edge_pair);
            if (iter != edge_pair_vec.end()) {return;}  // skip if edge is found
            
            // record edge if force not yet computed
            edge_pair_vec.push_back(edge_pair);

            // calculate contact point to edge
            EigenVector3D delta_pos_p1p2_j = pos_p2_j - pos_p1_j;
            double relprojection_p1p2_ij_val = (pos_i - pos_p1_j).dot(delta_pos_p1p2_j) / delta_pos_p1p2_j.dot(delta_pos_p1p2_j);
            contact_ij = delta_pos_p1p2_j*relprojection_p1p2_ij_val + pos_p1_j;

        }

        // calculate distance from center to p2-p0 edge
        EigenVector3D delta_pos_p2p0_j = pos_p0_j - pos_p2_j;
        double relprojection_p2p0_ji_val = (pos_i - pos_p2_j).dot(delta_pos_p2p0_j) / delta_pos_p2p0_j.dot(delta_pos_p2p0_j);
        EigenVector3D projection_p2p0_ji = delta_pos_p2p0_j*relprojection_p2p0_ji_val + pos_p2_j;
        double dist_p2p0_ji_mag = (pos_i - projection_p2p0_ji).norm();

        // calculate criteria for p2-p0 edge collision
        bool is_p2p0_collision = ((dist_p2p0_ji_mag + 1e-8) < radius_i);

        // determine contact point if p2-p0 edge collision
        if (is_p2p0_collision)
        {
            
            // skip if edge has been tested before
            // avoid double counting of forces
            std::pair<int, int> edge_pair = {mesh_j.pid_p2, mesh_j.pid_p0};
            if (mesh_j.pid_p2 > mesh_j.pid_p0) {edge_pair = {mesh_j.pid_p0, mesh_j.pid_p2};}  // let first < second for consistency
            auto iter = std::find(edge_pair_vec.begin(), edge_pair_vec.end(), edge_pair);
            if (iter != edge_pair_vec.end()) {return;}  // skip if edge is found
            
            // record edge if force not yet computed
            edge_pair_vec.push_back(edge_pair);

            // calculate contact point to edge
            EigenVector3D delta_pos_p2p0_j = pos_p0_j - pos_p2_j;
            double relprojection_p2p0_ij_val = (pos_i - pos_p2_j).dot(delta_pos_p2p0_j) / delta_pos_p2p0_j.dot(delta_pos_p2p0_j);
            contact_ij = delta_pos_p2p0_j*relprojection_p2p0_ij_val + pos_p2_j;

        }

        // calculate criteria for edge collision
        bool is_edge_collision = is_p0p1_collision || is_p1p2_collision || is_p2p0_collision;

        // if not edge collision, then vertex collision
        if (!is_edge_collision)
        {

            // check if p0 collision
            double dist_p0_ij_mag = (pos_p0_j - pos_i).norm();
            bool is_vertex_p0_collision = dist_p0_ij_mag < radius_i;

            // get contact point if p0 collision
            if (is_vertex_p0_collision)
            {
                auto iter = std::find(vertex_vec.begin(), vertex_vec.end(), mesh_j.pid_p0);
                if (iter != vertex_vec.end()) {return;}  // skip if vertex is found
                vertex_vec.push_back(mesh_j.pid_p0);
                contact_ij = pos_p0_j;
            }

            // check if p1 collision
            double dist_p1_ij_mag = (pos_p1_j - pos_i).norm();
            bool is_vertex_p1_collision = dist_p1_ij_mag < radius_i;

            // get contact point if p1 collision
            if (is_vertex_p1_collision)
            {
                auto iter = std::find(vertex_vec.begin(), vertex_vec.end(), mesh_j.pid_p1);
                if (iter != vertex_vec.end()) {return;}  // skip if vertex is found
                vertex_vec.push_back(mesh_j.pid_p1);
                contact_ij = pos_p1_j;
            }

            // check if p2 collision
            double dist_p2_ij_mag = (pos_p2_j - pos_i).norm();
            bool is_vertex_p2_collision = dist_p2_ij_mag < radius_i;

            // get contact point if p2 collision
            if (is_vertex_p2_collision)
            {
                auto iter = std::find(vertex_vec.begin(), vertex_vec.end(), mesh_j.pid_p2);
                if (iter != vertex_vec.end()) {return;}  // skip if vertex is found
                vertex_vec.push_back(mesh_j.pid_p2);
                contact_ij = pos_p2_j;
            }

            // overall criteria for vertex collision
            bool is_vertex_collision = is_vertex_p0_collision || is_vertex_p1_collision || is_vertex_p2_collision;

            // skip if no collision
            if (!is_vertex_collision)
            {
                overlap_tangent_map.erase(collision_pair);
                return;
            }

        }

    }

    // end of contact point calculation

    // calculate normal vector
    EigenVector3D delta_pos_ij = contact_ij - pos_i;
    double delta_pos_ij_mag = delta_pos_ij.norm();
    EigenVector3D normal_ij = delta_pos_ij/delta_pos_ij_mag;

    // get particle velocities
    EigenVector3D vel_i = sphere_i.velocity;
    EigenVector3D angvel_i = sphere_i.angularvelocity;

    // get velocity at contact point

    // get wall velocities
    EigenVector3D vel_trn_j = meshgroup_ptr->velocity_translate;
    double angvel_rot_j = meshgroup_ptr->angularvelocity_rotate;
    EigenVector3D pos_rotaxis_p0_j = meshgroup_ptr->position_rotateaxis_begin;
    EigenVector3D pos_rotaxis_p1_j = meshgroup_ptr->position_rotateaxis_end;

    // get axis of rotation
    EigenVector3D delta_pos_rotaxis_p0p1_j = pos_rotaxis_p1_j - pos_rotaxis_p0_j;
    double delta_pos_rotaxis_p0p1_j_mag = delta_pos_rotaxis_p0p1_j.norm();
    EigenVector3D unit_rotaxis_p0p1_j = delta_pos_rotaxis_p0p1_j/delta_pos_rotaxis_p0p1_j_mag;
    if (delta_pos_rotaxis_p0p1_j_mag < 1e-8)
    {
        unit_rotaxis_p0p1_j = {0., 0., 0.};
    }

    // calculate rotational velocity at contact point
    EigenVector3D delta_pos_p0_contact = contact_ij - pos_rotaxis_p0_j;  // p0 to contact point
    double delta_pos_p0_close_val = delta_pos_p0_contact.dot(unit_rotaxis_p0p1_j);  // p0 to "close" point (projection of contact point on rotation axis)
    EigenVector3D delta_pos_p0_close = delta_pos_p0_close_val*unit_rotaxis_p0p1_j;
    EigenVector3D delta_close_contact = delta_pos_p0_contact - delta_pos_p0_close;
    EigenVector3D vel_rot_j = angvel_rot_j*unit_rotaxis_p0p1_j.cross(delta_close_contact);

    // calculate velocity at contact point
    EigenVector3D vel_j = vel_trn_j + vel_rot_j;

    // get material type
    int material_i = sphere_i.mid;
    int material_j = 0; // TODO

    // get collision properties
    double spring_normal = spring_normal_ptr->get_value(material_i, material_j);
    double spring_tangent = spring_tangent_ptr->get_value(material_i, material_j);
    double damping_normal = damping_normal_ptr->get_value(material_i, material_j);
    double damping_tangent = damping_tangent_ptr->get_value(material_i, material_j);
    double friction_sliding = friction_sliding_ptr->get_value(material_i, material_j);
    double friction_rolling = friction_rolling_ptr->get_value(material_i, material_j);

    // get tangential overlap
    double overlap_tangent_ij_val = get_overlap_tangent_value(collision_pair);

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
    overlap_tangent_map[collision_pair] += relvel_tangent_ij_val*dt;
    
    // add forces on sphere i
    // apply Newton's third law to get forces on j
    spheregroup_ptr->sphere_vec[indx_i].force += force_collision_ij;
    // meshgroup_ptr->force += -force_collision_ij; // TODO

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
    spheregroup_ptr->sphere_vec[indx_i].moment += moment_collision_ij + moment_friction_ij;
    // sphere_j_ptr->moment_vec[tid_j] += moment_collision_ji + moment_friction_ji;

}

}

#endif
