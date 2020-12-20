////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <sbpl_collision_checking/self_collision_model.h>

// system includes
#include <leatherman/print.h>
#include <smpl/geometry/triangle.h>
#include <smpl/geometry/intersect.h>

// project includes
#include "collision_operations.h"

namespace smpl {
namespace collision {

static const char* SCM_LOGGER = "self";

SelfCollisionModel::SelfCollisionModel(
    OccupancyGrid* grid,
    const RobotCollisionModel* rcm,
    const AttachedBodiesCollisionModel* ab_model)
:
    m_grid(grid),
    m_rcm(rcm),
    m_abcm(ab_model),
    m_rcs(rcm),
    m_abcs(ab_model, &m_rcs),
    m_gidx(-1),
    m_voxels_indices(),
    m_ab_voxels_indices(),
    m_checked_spheres_states(),
    m_checked_attached_body_spheres_states(),
    m_checked_attached_body_robot_spheres_states(),
    m_acm(),
    m_padding(0.0),
#if SCDL_USE_META_TREE
    m_model_state_map(),
    m_root_models(),
    m_root_model_pointers(),
    m_meta_model(),
    m_meta_state(),
#endif
    m_q(),
    m_vq()
{
    initAllowedCollisionMatrix();
}

/// Seed the allowed collision matrix with pairs of adjacent links.
void SelfCollisionModel::initAllowedCollisionMatrix()
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Create adjacent link entries in the allowed collision matrix");
    for (size_t lidx = 0; lidx < m_rcm->linkCount(); ++lidx) {
        const std::string& link_name = m_rcm->linkName(lidx);
        if (!m_acm.hasEntry(link_name)) {
            m_acm.setEntry(link_name, false);
        }

        int pjidx = m_rcm->linkParentJointIndex(lidx);
        if (pjidx != 0) {
            const int plidx = m_rcm->jointParentLinkIndex(pjidx);
            const std::string& parent_link_name = m_rcm->linkName(plidx);
            if (!m_acm.hasEntry(parent_link_name)) {
                m_acm.setEntry(parent_link_name, false);
            }

            m_acm.setEntry(link_name, parent_link_name, true);
        }

        for (int cjidx : m_rcm->linkChildJointIndices(lidx)) {
            int clidx = m_rcm->jointChildLinkIndex(cjidx);
            const std::string& child_link_name = m_rcm->linkName(clidx);
            if (!m_acm.hasEntry(child_link_name)) {
                m_acm.setEntry(child_link_name, false);
            }

            m_acm.setEntry(link_name, child_link_name, true);
        }
    }
    // NOTE: no need to update checked sphere indices here, they will be updated
    // when the first request with a valid group index is received
}

/// Check that the input states are related to the collision models passed to
/// the constructor.
bool SelfCollisionModel::checkCommonInputs(
    const RobotCollisionState& state,
    const AttachedBodiesCollisionState& ab_state,
    const int gidx) const
{
    if (state.model() != m_rcm) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Robot Collision State is for another Robot Collision Model");
        return false;
    }
    if (ab_state.model() != m_abcm) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Attached Bodies Collision State is for another Attached Bodies Collision Model");
        return false;
    }

    if (gidx < 0 || gidx >= m_rcm->groupCount() || gidx >= m_abcm->groupCount()) {
        ROS_ERROR_NAMED(SCM_LOGGER, "self collision check is for non-existent group");
        return false;
    }

    return true;
}

/// Prepare internal collision states with a query state and group
void SelfCollisionModel::prepareState(
    int gidx,
    const double* state)
{
    updateGroup(gidx);
    copyState(state);
    updateVoxelsStates();
}

SelfCollisionModel::~SelfCollisionModel()
{
}

const AllowedCollisionMatrix&
SelfCollisionModel::allowedCollisionMatrix() const
{
    return m_acm;
}

/// Update the allowed collision matrix, removing entries specified as NEVER and
/// adding entries specified as ALWAYS in the input matrix
void SelfCollisionModel::updateAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Update allowed collision matrix");
    std::vector<std::string> all_entries;

    acm.getAllEntryNames(all_entries);

    collision_detection::AllowedCollision::Type type;
    for (size_t i = 0; i < all_entries.size(); ++i) {
        for (size_t j = i + 1; j < all_entries.size(); ++j) {
            const std::string& entry1 = all_entries[i];
            const std::string& entry2 = all_entries[j];
            if (acm.getEntry(entry1, entry2, type)) {
                if (type == collision_detection::AllowedCollision::NEVER) 
                {
                    m_acm.setEntry(entry1, entry2, false);
                }
                else if (type == collision_detection::AllowedCollision::ALWAYS || type == collision_detection::AllowedCollision::CONDITIONAL)
                {
                    m_acm.setEntry(entry1, entry2, true);
                }
                
            }
        }
    }
    updateCheckedSpheresIndices();
}

void SelfCollisionModel::setAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Overwrite allowed collision matrix");
    m_acm = acm;
    updateCheckedSpheresIndices();
}

/// Set the padding to be applied to spheres. No padding is applied to voxels
/// models.
void SelfCollisionModel::setPadding(double padding)
{
    m_padding = padding;
}

void SelfCollisionModel::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    (void)m_rcs.setWorldToModelTransform(transform);
}

bool SelfCollisionModel::checkCollision(
    const RobotCollisionState& state,
    const AttachedBodiesCollisionState& ab_state,
    const int gidx,
    double& dist)
{
    if (!checkCommonInputs(state, ab_state, gidx)) {
        return false;
    }

    prepareState(gidx, state.getJointVarPositions());

    if (!checkRobotVoxelsStateCollisions(dist) ||
        !checkAttachedBodyVoxelsStateCollisions(dist) ||
        !checkRobotSpheresStateCollisions(dist) ||
        !checkAttachedBodySpheresStateCollisions(dist))
    {
        return false;
    }

    return true;
}

bool SelfCollisionModel::checkCollision(
    const RobotCollisionState& state,
    const AttachedBodiesCollisionState& ab_state,
    const AllowedCollisionsInterface& aci,
    const int gidx,
    double& dist)
{
    if (!checkCommonInputs(state, ab_state, gidx)) {
        return false;
    }

    prepareState(gidx, state.getJointVarPositions());

    if (!checkRobotVoxelsStateCollisions(dist) ||
        !checkAttachedBodyVoxelsStateCollisions(dist) ||
        !checkRobotSpheresStateCollisions(aci, dist) ||
        !checkAttachedBodySpheresStateCollisions(aci, dist))
    {
        return false;
    }

    return true;
}

bool SelfCollisionModel::checkMotionCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const RobotMotionCollisionModel& rmcm,
    const std::vector<double>& start,
    const std::vector<double>& finish,
    const int gidx,
    double& dist)
{
    const double res = 0.05;
    MotionInterpolation interp(m_rcm);
    rmcm.fillMotionInterpolation(start, finish, res, interp);

    RobotState interm;
    for (int i = 0; i < interp.waypointCount(); ++i) {
        interp.interpolate(i, interm);
        state.setJointVarPositions(interm.data());
        if (!checkCollision(state, ab_state, gidx, dist)) {
            return false;
        }
    }
    return true;
}

bool SelfCollisionModel::checkMotionCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const AllowedCollisionsInterface& aci,
    const RobotMotionCollisionModel& rmcm,
    const std::vector<double>& start,
    const std::vector<double>& finish,
    const int gidx,
    double& dist)
{
    const double res = 0.05;
    MotionInterpolation interp(m_rcm);
    rmcm.fillMotionInterpolation(start, finish, res, interp);

    RobotState interm;
    for (int i = 0; i < interp.waypointCount(); ++i) {
        interp.interpolate(i, interm);
        state.setJointVarPositions(interm.data());
        if (!checkCollision(state, ab_state, aci, gidx, dist)) {
            return false;
        }
    }
    return true;
}

double SelfCollisionModel::collisionDistance(
    const RobotCollisionState& state,
    const AttachedBodiesCollisionState& ab_state,
    const int gidx)
{
    if (!checkCommonInputs(state, ab_state, gidx)) {
        return false;
    }

    prepareState(gidx, state.getJointVarPositions());

    double tmp;

    double d = robotVoxelsCollisionDistance();
    tmp = attachedBodyVoxelsCollisionDistance();
    if (tmp < d) {
        d = tmp;
    }
    tmp = robotSpheresCollisionDistance();
    if (tmp < d) {
        d = tmp;
    }
    tmp = attachedBodySpheresCollisionDistance();
    if (tmp < d) {
        d = tmp;
    }

    return d;
}

double SelfCollisionModel::collisionDistance(
    const RobotCollisionState& state,
    const AttachedBodiesCollisionState& ab_state,
    const AllowedCollisionsInterface& aci,
    const int gidx)
{
    if (!checkCommonInputs(state, ab_state, gidx)) {
        return false;
    }

    prepareState(gidx, state.getJointVarPositions());

    double tmp;

    double d = robotVoxelsCollisionDistance();
    tmp = attachedBodyVoxelsCollisionDistance();
    if (tmp < d) {
        d = tmp;
    }
    tmp = robotSpheresCollisionDistance(aci);
    if (tmp < d) {
        d = tmp;
    }
    tmp = attachedBodySpheresCollisionDistance(aci);
    if (tmp < d) {
        d = tmp;
    }

    return d;
}

bool SelfCollisionModel::collisionDetails(
    const RobotCollisionState& state,
    const AttachedBodiesCollisionState& ab_state,
    const int gidx,
    CollisionDetails& details)
{
    if (!checkCommonInputs(state, ab_state, gidx)) {
        return false;
    }

    prepareState(gidx, state.getJointVarPositions());

    details.voxels_collision_count = 0;
    details.sphere_collision_count = 0;
    bool res = true;
    res &= getRobotVoxelsStateCollisionDetails(details);
    res &= getAttachedBodyVoxelsStateCollisionDetails(details);
    res &= getRobotSpheresStateCollisionDetails(details);
    res &= getAttachedBodySpheresStateCollisionDetails(details);
    return res;
}

bool SelfCollisionModel::collisionDetails(
    const RobotCollisionState& state,
    const AttachedBodiesCollisionState& ab_state,
    const AllowedCollisionsInterface& aci,
    const int gidx,
    CollisionDetails& details)
{
    if (!checkCommonInputs(state, ab_state, gidx)) {
        return false;
    }

    prepareState(gidx, state.getJointVarPositions());

    double dist;
    details.voxels_collision_count = 0;
    details.sphere_collision_count = 0;
    bool res = true;
    res &= getRobotVoxelsStateCollisionDetails(details);
    res &= getAttachedBodyVoxelsStateCollisionDetails(details);
    res &= getRobotSpheresStateCollisionDetails(aci, details);
    res &= getAttachedBodySpheresStateCollisionDetails(aci, details);
    return res;
}

/// Switch to checking for a new collision group
///
/// Removes voxels from groups that are inside the new collision group and add
/// voxels that are outside the new collision group. Also updates the cached
/// set of checked sphere indices for checking against the default collision
/// matrix
void SelfCollisionModel::updateGroup(int gidx)
{
    if (gidx == m_gidx) {
        return;
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "Update Self Collision Model from group %d to group %d", m_gidx, gidx);

    // switch to new voxels state context

    std::vector<int> old_ov_indices = m_voxels_indices;

    std::sort(old_ov_indices.begin(), old_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "Old outside voxels indices: %s", to_string(old_ov_indices).c_str());

    std::vector<int> new_ov_indices = m_rcs.groupOutsideVoxelsStateIndices(gidx);
    std::sort(new_ov_indices.begin(), new_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "New outside voxels indices: %s", to_string(new_ov_indices).c_str());

    // get the indices of the voxels states that were outside the group but are
    // now inside and must be removed
    std::vector<int> ovidx_rem;
    std::set_difference(
            old_ov_indices.begin(), old_ov_indices.end(),
            new_ov_indices.begin(), new_ov_indices.end(),
            std::back_inserter(ovidx_rem));
    ROS_DEBUG_NAMED(SCM_LOGGER, "ovidx_rem: %s", to_string(ovidx_rem).c_str());

    // get the indices of the voxels states that were inside the group but are
    // now outside and must be inserted
    std::vector<int> ovidx_ins;
    std::set_difference(
            new_ov_indices.begin(), new_ov_indices.end(),
            old_ov_indices.begin(), old_ov_indices.end(),
            std::back_inserter(ovidx_ins));
    ROS_DEBUG_NAMED(SCM_LOGGER, "ovidx_ins: %s", to_string(ovidx_ins).c_str());

    // gather the voxels to be removed
    std::vector<Eigen::Vector3d> v_rem;
    for (int vsidx : ovidx_rem) {
        const CollisionVoxelsState& vs = m_rcs.voxelsState(vsidx);
        v_rem.insert(v_rem.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // gather the voxels to be inserted
    std::vector<Eigen::Vector3d> v_ins;
    for (int vsidx : ovidx_ins) {
        const CollisionVoxelsState& vs = m_rcs.voxelsState(vsidx);
        v_ins.insert(v_ins.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // insert/remove the voxels
    if (!v_rem.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Remove %zu voxels from old voxels models", v_rem.size());
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Insert %zu voxels from new voxels models", v_ins.size());
        m_grid->addPointsToField(v_ins);
    }

    // prepare voxels indices

    m_voxels_indices = std::move(new_ov_indices);

    std::vector<int> old_ab_ov_indices = m_ab_voxels_indices;
    std::sort(old_ab_ov_indices.begin(), old_ab_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "Old attached body outside voxels indices: %s", to_string(old_ab_ov_indices).c_str());

    std::vector<int> new_ab_ov_indices = m_abcs.groupOutsideVoxelsStateIndices(gidx);
    std::sort(new_ab_ov_indices.begin(), new_ab_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "New attached body outside voxels indices: %s", to_string(new_ab_ov_indices).c_str());

    std::vector<int> abovidx_rem;
    std::set_difference(
            old_ab_ov_indices.begin(), old_ab_ov_indices.end(),
            new_ab_ov_indices.begin(), new_ab_ov_indices.end(),
            std::back_inserter(abovidx_rem));
    ROS_DEBUG_NAMED(SCM_LOGGER, "abovidx_rem: %s", to_string(abovidx_rem).c_str());

    std::vector<int> abovidx_ins;
    std::set_difference(
            new_ab_ov_indices.begin(), new_ab_ov_indices.end(),
            old_ab_ov_indices.begin(), old_ab_ov_indices.end(),
            std::back_inserter(abovidx_ins));
    ROS_DEBUG_NAMED(SCM_LOGGER, "ovidx_ins: %s", to_string(abovidx_ins).c_str());

    v_rem.clear();
    for (int vsidx : abovidx_rem) {
        const CollisionVoxelsState& vs = m_abcs.voxelsState(vsidx);
        v_rem.insert(v_rem.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // gather the voxels to be inserted
    v_ins.clear();
    for (int vsidx : abovidx_ins) {
        const CollisionVoxelsState& vs = m_abcs.voxelsState(vsidx);
        v_ins.insert(v_ins.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // insert/remove the voxels
    if (!v_rem.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Remove %zu voxels from old voxels models", v_rem.size());
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Insert %zu voxels from new voxels models", v_ins.size());
        m_grid->addPointsToField(v_ins);
    }

    m_ab_voxels_indices = std::move(new_ab_ov_indices);

#if SCDL_USE_META_TREE
    // map from meta sphere leaf model to its corresponding collision sphere root state
    m_model_state_map.clear();

    // gather the root collision sphere models for each link in the group;
    const auto& spheres_state_indices = m_rcs.groupSpheresStateIndices(gidx);
    m_root_models.resize(spheres_state_indices.size());
    m_root_model_pointers.resize(spheres_state_indices.size());
    for (size_t i = 0; i < m_root_models.size(); ++i) {
        m_root_model_pointers[i] = &m_root_models[i];
        const int ssidx = spheres_state_indices[i];
        // update the model with the position of the root spheres state
        const auto& ss = m_rcs.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        CollisionSphereModel& sm = m_root_models[i];
        sm.radius = s->model->radius;
        sm.priority = s->model->priority;
        m_model_state_map[m_root_model_pointers[i]] = s;
    }

    m_meta_model.link_index = -1; // not attached to any link

    // create a state for the model
    m_meta_state.model = &m_meta_model;
    m_meta_state.index = -1; // no position in the robot state
#endif

    // activate the group
    m_gidx = gidx;

    // prepare the set of spheres states that should be checked for collision
    updateCheckedSpheresIndices();
}

void SelfCollisionModel::copyState(const double* state)
{
    (void)m_rcs.setJointVarPositions(state);
}

/// Lazily update the state of the occupancy grid, representing areas filled by
/// voxels models, when voxels state changes are detected in the robot collision
/// state (and attached bodies collision state)
void SelfCollisionModel::updateVoxelsStates()
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Update voxels states");
    // gather all changed voxels before updating so as to impose only a single
    // distance field update
    auto& v_rem = m_v_rem; v_rem.clear();
    auto& v_ins = m_v_ins; v_ins.clear();

    for (int vsidx : m_voxels_indices) {
        if (m_rcs.voxelsStateDirty(vsidx)) {
            const CollisionVoxelsState& voxels_state = m_rcs.voxelsState(vsidx);

            // copy over voxels to be removed before updating
            const size_t prev_size = v_rem.size();
            v_rem.insert(
                    v_rem.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());
            const size_t curr_size = v_rem.size();

            m_rcs.updateVoxelsState(vsidx);

            // copy over voxels to be inserted
            v_ins.insert(
                    v_ins.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            ROS_DEBUG_NAMED(SCM_LOGGER, "  Update Occupancy Grid with change to Collision Voxels State (%zu displaced)", curr_size - prev_size);
        }
    }

    // same method as above against attached bodies collision state
    for (int vsidx : m_ab_voxels_indices) {
        if (m_abcs.voxelsStateDirty(vsidx)) {
            const CollisionVoxelsState& voxels_state =
                    m_abcs.voxelsState(vsidx);

            // copy over voxels to be removed before updating
            const size_t prev_size = v_rem.size();
            v_rem.insert(
                    v_rem.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());
            const size_t curr_size = v_rem.size();

            m_abcs.updateVoxelsState(vsidx);

            // copy over voxels to be inserted
            v_ins.insert(
                    v_ins.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            ROS_DEBUG_NAMED(SCM_LOGGER, "  Update Occupancy Grid with change to Collision Voxels State (%zu displaced)", curr_size - prev_size);
        }
    }

    // update occupancy grid with new voxel data
    if (!v_rem.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Remove %zu voxels", v_rem.size());
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Insert %zu voxels", v_ins.size());
        m_grid->addPointsToField(v_ins);
    }
}

bool SelfCollisionModel::checkRobotVoxelsStateCollisions(double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check robot links against voxels states");

#if SCDL_USE_META_TREE
    updateMetaSphereTrees();
#endif

    auto& q = m_vq;
    q.clear();

#if SCDL_USE_META_TREE
    q.push_back(m_meta_state.spheres.root());
#else
    for (const int ssidx : m_rcs.groupSpheresStateIndices(m_gidx)) {
        const auto& ss = m_rcs.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        q.push_back(s);
    }
#endif

    return CheckVoxelsCollisions(m_rcs, q, *m_grid, m_padding, dist);
}

bool SelfCollisionModel::checkAttachedBodyVoxelsStateCollisions(
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check attached bodies against voxels states");

    auto& q = m_vq;
    q.clear();

    for (const int ssidx : m_abcs.groupSpheresStateIndices(m_gidx)) {
        const auto& ss = m_abcs.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        q.push_back(s);
    }

    return CheckVoxelsCollisions(m_abcs, q, *m_grid, m_padding, dist);
}

bool SelfCollisionModel::checkRobotSpheresStateCollisions(double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check robot links vs robot links");

    for (const auto& ss_pair : m_checked_spheres_states) {
        int ss1i = ss_pair.first;
        int ss2i = ss_pair.second;
        auto& ss1 = m_rcs.spheresState(ss1i);
        auto& ss2 = m_rcs.spheresState(ss2i);

        if (!checkSpheresStateCollision(
                m_rcs, m_rcs, ss1i, ss2i, ss1, ss2, dist))
        {
            return false;
        }
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "No spheres collisions");
    return true;
}

bool SelfCollisionModel::checkRobotSpheresStateCollisions(
    const AllowedCollisionsInterface& aci,
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check robot links vs robot links");

    const auto& group_link_indices = m_rcm->groupLinkIndices(m_gidx);
    for (int l1 = 0; l1 < group_link_indices.size(); ++l1) {
        const int lidx1 = group_link_indices[l1];
        if (!m_rcm->hasSpheresModel(lidx1)) {
            continue;
        }

        auto& l1_name = m_rcm->linkName(lidx1);
        for (int l2 = l1 + 1; l2 < group_link_indices.size(); ++l2) {
            const int lidx2 = group_link_indices[l2];
            if (!m_rcm->hasSpheresModel(lidx2)) {
                continue;
            }
            auto& l2_name = m_rcm->linkName(lidx2);

            AllowedCollision::Type type;
            if (aci.getEntry(l2_name, l1_name, type) &&
                type == AllowedCollision::Type::ALWAYS)
            {
                // collisions allowed between this pair of links
                continue;
            }

            const int ss1i = m_rcs.linkSpheresStateIndex(lidx1);
            const int ss2i = m_rcs.linkSpheresStateIndex(lidx2);
            auto& ss1 = m_rcs.spheresState(ss1i);
            auto& ss2 = m_rcs.spheresState(ss2i);
            if (!checkSpheresStateCollision(
                    m_rcs, m_rcs, ss1i, ss2i, ss1, ss2, dist))
            {
                return false;
            }
        }
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "No spheres collisions");
    return true;
}

bool SelfCollisionModel::checkAttachedBodySpheresStateCollisions(
    double& dist)
{
    ROS_DEBUG(SCM_LOGGER, "Check attached bodies vs attached bodies");

    for (const auto& ss_pair : m_checked_attached_body_spheres_states) {
        int ss1i = ss_pair.first;
        int ss2i = ss_pair.second;
        const CollisionSpheresState& ss1 = m_abcs.spheresState(ss1i);
        const CollisionSpheresState& ss2 = m_abcs.spheresState(ss2i);

        if (!checkSpheresStateCollision(
                m_abcs, m_abcs, ss1i, ss2i, ss1, ss2, dist))
        {
            return false;
        }
    }

    if (!checkRobotAttachedBodySpheresStateCollisions(dist)) {
        return false;
    }

    return true;
}

bool SelfCollisionModel::checkAttachedBodySpheresStateCollisions(
    const AllowedCollisionsInterface& aci,
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check attached bodies vs attached bodies");
    const auto& group_body_indices = m_abcm->groupLinkIndices(m_gidx);
    for (int b1 = 0; b1 < group_body_indices.size(); ++b1) {
        const int bidx1 = group_body_indices[b1];
        if (!m_abcm->hasSpheresModel(bidx1)) {
            continue;
        }

        const std::string& b1_name = m_abcm->attachedBodyName(bidx1);
        for (int b2 = b1 + 1; b2 < group_body_indices.size(); ++b2) {
            const int bidx2 = group_body_indices[b2];
            if (!m_abcm->hasSpheresModel(bidx2)) {
                continue;
            }

            const std::string& b2_name = m_abcm->attachedBodyName(bidx2);

            AllowedCollision::Type type;
            if (aci.getEntry(b1_name, b2_name, type) &&
                type == AllowedCollision::Type::ALWAYS)
            {
                // collisions between this pair of links
                continue;
            }

            const int ss1i = m_abcs.attachedBodySpheresStateIndex(bidx1);
            const int ss2i = m_abcs.attachedBodySpheresStateIndex(bidx2);
            const CollisionSpheresState& ss1 = m_abcs.spheresState(ss1i);
            const CollisionSpheresState& ss2 = m_abcs.spheresState(ss2i);
            if (!checkSpheresStateCollision(
                    m_abcs, m_abcs, ss1i, ss2i, ss1, ss2, dist))
            {
                return false;
            }
        }
    }

    if (!checkRobotAttachedBodySpheresStateCollisions(aci, dist)) {
        return false;
    }

    return true;
}

bool SelfCollisionModel::checkRobotAttachedBodySpheresStateCollisions(
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check attached bodies vs robot links");

    for (const auto& ss_pair : m_checked_attached_body_robot_spheres_states) {
        int ss1i = ss_pair.first;
        int ss2i = ss_pair.second;
        const CollisionSpheresState& ss1 = m_abcs.spheresState(ss1i);
        const CollisionSpheresState& ss2 = m_rcs.spheresState(ss2i);

        if (!checkSpheresStateCollision(
                m_abcs, m_rcs, ss1i, ss2i, ss1, ss2, dist))
        {
            return false;
        }
    }

    return true;
}

bool SelfCollisionModel::checkRobotAttachedBodySpheresStateCollisions(
    const AllowedCollisionsInterface& aci,
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check attached bodies vs robot links");
    const auto& group_link_indices = m_rcm->groupLinkIndices(m_gidx);
    const auto& group_body_indices = m_abcm->groupLinkIndices(m_gidx);
    for (int b1 = 0; b1 < group_body_indices.size(); ++b1) {
        const int bidx = group_body_indices[b1];
        if (!m_abcm->hasSpheresModel(bidx)) {
            continue;
        }

        const std::string& body_name = m_abcm->attachedBodyName(bidx);
        for (int l1 = 0; l1 < group_link_indices.size(); ++l1) {
            const int lidx = group_link_indices[l1];
            if (!m_rcm->hasSpheresModel(lidx)) {
                continue;
            }

            const std::string& link_name = m_rcm->linkName(lidx);

            AllowedCollision::Type type;
            if (aci.getEntry(body_name, link_name, type) &&
                type == AllowedCollision::Type::ALWAYS)
            {
                // collisions between this pair of links
                continue;
            }

            const int ss1i = m_abcs.attachedBodySpheresStateIndex(bidx);
            const int ss2i = m_rcs.linkSpheresStateIndex(lidx);
            const CollisionSpheresState& ss1 = m_abcs.spheresState(ss1i);
            const CollisionSpheresState& ss2 = m_rcs.spheresState(ss2i);
            if (!checkSpheresStateCollision(
                    m_abcs, m_rcs, ss1i, ss2i, ss1, ss2, dist))
            {
                return false;
            }
        }
    }

    return true;
}

bool CheckGeometryCollision(
    const CollisionGeometry& shape1,
    const CollisionGeometry& shape2,
    int shape_index1,
    int shape_index2,
    const Eigen::Affine3d& pose1,
    const Eigen::Affine3d& pose2)
{
    Eigen::Affine3d p1 = pose1 * shape1.offset;
    Eigen::Affine3d p2 = pose2 * shape2.offset;

    if (shape1.shape->type == ShapeType::Mesh &&
        shape2.shape->type == ShapeType::Mesh)
    {
        auto* m1 = static_cast<const MeshShape*>(shape1.shape);
        auto* m2 = static_cast<const MeshShape*>(shape2.shape);
        auto t11 = m1->triangles[3 * shape_index1 + 0];
        auto t12 = m1->triangles[3 * shape_index1 + 1];
        auto t13 = m1->triangles[3 * shape_index1 + 2];

        auto t21 = m2->triangles[3 * shape_index2 + 0];
        auto t22 = m2->triangles[3 * shape_index2 + 1];
        auto t23 = m2->triangles[3 * shape_index2 + 2];

        Eigen::Vector3d u1, u2, u3, v1, v2, v3;
        u1.x() = m1->vertices[3 * t11 + 0];
        u1.y() = m1->vertices[3 * t11 + 1];
        u1.z() = m1->vertices[3 * t11 + 2];

        u2.x() = m1->vertices[3 * t12 + 0];
        u2.y() = m1->vertices[3 * t12 + 1];
        u2.z() = m1->vertices[3 * t12 + 2];

        u3.x() = m1->vertices[3 * t13 + 0];
        u3.y() = m1->vertices[3 * t13 + 1];
        u3.z() = m1->vertices[3 * t13 + 2];

        v1.x() = m2->vertices[3 * t21 + 0];
        v1.y() = m2->vertices[3 * t21 + 1];
        v1.z() = m2->vertices[3 * t21 + 2];

        v2.x() = m2->vertices[3 * t22 + 0];
        v2.y() = m2->vertices[3 * t22 + 1];
        v2.z() = m2->vertices[3 * t22 + 2];

        v3.x() = m2->vertices[3 * t23 + 0];
        v3.y() = m2->vertices[3 * t23 + 1];
        v3.z() = m2->vertices[3 * t23 + 2];

        smpl::geometry::Triangle t1, t2;
        t1.a = p1 * u1;
        t1.b = p1 * u2;
        t1.c = p1 * u3;
        t2.a = p2 * v1;
        t2.b = p2 * v2;
        t2.c = p2 * v3;
        return !smpl::geometry::Intersects(t1, t2, 1e-4);
    }
    return true;
}

/// \tparam StateA RobotCollisionState or AttachedBodiesCollisionState
/// \tparam StateB RobotCollisionState or AttachedBodiesCollisionState
/// \param ss1i The index of the first spheres state
/// \param ss2i The index of the second spheres state
/// \param ss1 The first spheres state
/// \param ss2 The second spheres state
/// \param[out] The squared distance between the first two spheres that were
///     found in collision; unmodified if no collision was found
template <typename StateA, typename StateB>
bool SelfCollisionModel::checkSpheresStateCollision(
    StateA& stateA,
    StateB& stateB,
    int ss1i,
    int ss2i,
    const CollisionSpheresState& ss1,
    const CollisionSpheresState& ss2,
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check spheres state collision");
    auto sqrd = [](double d) { return d * d; };

    // assertion: both collision spheres are updated when they are removed from the stack
    stateA.updateSphereState(SphereIndex(ss1i, ss1.spheres.root()->index()));
    stateB.updateSphereState(SphereIndex(ss2i, ss2.spheres.root()->index()));

    auto& q = m_q;
    q.clear();
    q.push_back(std::make_pair(ss1.spheres.root(), ss2.spheres.root()));
    while (!q.empty()) {
        // get the next pair of spheres to check
        const CollisionSphereState *s1s, *s2s;
        std::tie(s1s, s2s) = q.back();
        q.pop_back();

        const CollisionSphereModel* s1m = s1s->model;
        const CollisionSphereModel* s2m = s2s->model;

        ROS_DEBUG_NAMED(SCM_LOGGER, "Checking '%s' x '%s' collision", s1m->name.c_str(), s2m->name.c_str());

        Eigen::Vector3d dx = s2s->pos - s1s->pos;
        const double cd2 = dx.squaredNorm(); // center distance squared
        const double cr2 = sqrd(s1m->radius + s2m->radius); // combined radius squared

        if (cd2 > cr2) {
            // no collision between spheres -> back out
            continue;
        }

        if (s1s->isLeaf() && s2s->isLeaf()) {
            // collision found! check acm
            collision_detection::AllowedCollision::Type type;
            if (m_acm.getEntry(s1m->name, s2m->name, type)) {
                if (type != collision_detection::AllowedCollision::ALWAYS) {
                    ROS_DEBUG_NAMED(SCM_LOGGER, "  *collision* '%s' x '%s'", s1m->name.c_str(), s2m->name.c_str());
                    dist = cd2;
                    return false;
                }
            } else {
                ROS_DEBUG_NAMED(SCM_LOGGER, "  *collision* '%s' x '%s'", s1m->name.c_str(), s2m->name.c_str());
                dist = cd2;
                return false;
            }
            // collision between leaves is ok
            continue;
        }

        // choose a sphere node to split
        bool split1;
        if (s1s->isLeaf()) { // split sphere 2
            split1 = false;
        } else if (s2s->isLeaf()) { // split sphere 1
            split1 = true;
        } else { // choose a node to split
            // heuristic -> split the larger sphere to obtain more
            // information about the underlying surface, assuming the leaf
            // spheres are often about the same size
            if (s1m->radius > s2m->radius) {
                split1 = true;
            } else {
                split1 = false;
            }
        }

        if (split1) {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Splitting node '%s'", s1m->name.c_str());
            const CollisionSphereState* sl = s1s->left;
            const CollisionSphereState* sr = s1s->right;
            // update children positions
            stateA.updateSphereState(SphereIndex(ss1i, sl->index()));
            stateA.updateSphereState(SphereIndex(ss1i, sr->index()));

            // heuristic -> examine the pair of spheres that are closer together
            // first for a better chance at detecting collision

            // dist from split sphere's left child to other sphere
            const double cd2l2 = (s2s->pos - sl->pos).squaredNorm();
            // dist from split sphere's right child to other sphere
            const double cd2r2 = (s2s->pos - sr->pos).squaredNorm();

            if (cd2l2 < cd2r2) {
                // examine right child after the left child
                q.push_back(std::make_pair(sr, s2s));
                q.push_back(std::make_pair(sl, s2s));
            } else {
                // examine the left child after the right child
                q.push_back(std::make_pair(sl, s2s));
                q.push_back(std::make_pair(sr, s2s));
            }
        } else {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Splitting node '%s'", s2m->name.c_str());
            // equivalent comments from above
            const CollisionSphereState* sl = s2s->left;
            const CollisionSphereState* sr = s2s->right;

            stateB.updateSphereState(SphereIndex(ss2i, sl->index()));
            stateB.updateSphereState(SphereIndex(ss2i, sr->index()));

            double cd1l2 = (s1s->pos - sl->pos).squaredNorm();
            double cd1r2 = (s1s->pos - sr->pos).squaredNorm();

            if (cd1l2 < cd1r2) {
                q.push_back(std::make_pair(s1s, sr));
                q.push_back(std::make_pair(s1s, sl));
            } else {
                q.push_back(std::make_pair(s1s, sl));
                q.push_back(std::make_pair(s1s, sr));
            }
        }
    }
    ROS_DEBUG_NAMED(SCM_LOGGER, "queue exhaused");

    // queue exhaused = no collision found
    return true;
}

bool SelfCollisionModel::checkSpheresStateCollision(
    RobotCollisionState& stateA,
    RobotCollisionState& stateB,
    int ss1i,
    int ss2i,
    const CollisionSpheresState& ss1,
    const CollisionSpheresState& ss2,
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check spheres state collision");
    auto sqrd = [](double d) { return d * d; };

    // assertion: both collision spheres are updated when they are removed from the stack
    stateA.updateSphereState(SphereIndex(ss1i, ss1.spheres.root()->index()));
    stateB.updateSphereState(SphereIndex(ss2i, ss2.spheres.root()->index()));

    auto& q = m_q;
    q.clear();
    q.push_back(std::make_pair(ss1.spheres.root(), ss2.spheres.root()));
    while (!q.empty()) {
        // get the next pair of spheres to check
        const CollisionSphereState *s1s, *s2s;
        std::tie(s1s, s2s) = q.back();
        q.pop_back();

        const CollisionSphereModel* s1m = s1s->model;
        const CollisionSphereModel* s2m = s2s->model;

        ROS_DEBUG_NAMED(SCM_LOGGER, "Checking '%s' x '%s' collision", s1m->name.c_str(), s2m->name.c_str());

        Eigen::Vector3d dx = s2s->pos - s1s->pos;
        const double cd2 = dx.squaredNorm(); // center distance squared
        const double cr2 = sqrd(s1m->radius + s2m->radius); // combined radius squared

        if (cd2 > cr2) {
            // no collision between spheres -> back out
            continue;
        }

        if (s1s->isLeaf() && s2s->isLeaf()) {
            // collision found! check acm
            collision_detection::AllowedCollision::Type type;
            if (m_acm.getEntry(s1m->name, s2m->name, type)) {
                if (type != collision_detection::AllowedCollision::ALWAYS) {
                    ROS_DEBUG_NAMED(SCM_LOGGER, "  *collision* '%s' x '%s'", s1m->name.c_str(), s2m->name.c_str());
                    dist = cd2;
                    return false;
                }
            } else {
                if (s1s->model->geom && s2s->model->geom) {
                    // shape pose = pose of link * offset of shape
                    auto l1_index = s1s->parent_state->model->link_index;
                    auto l2_index = s2s->parent_state->model->link_index;
                    assert(stateA.linkTransformDirty(l1_index) == false);
                    assert(stateB.linkTransformDirty(l2_index) == false);

                    auto& l1_pose = stateA.linkTransform(l1_index);
                    auto& l2_pose = stateB.linkTransform(l2_index);
                    auto& pose1 = l1_pose;
                    auto& pose2 = l2_pose;
                    if (!CheckGeometryCollision(
                            *s1s->model->geom,
                            *s2s->model->geom,
                            s1s->model->shape_index,
                            s2s->model->shape_index,
                            pose1,
                            pose2))
                    {
                        dist = cd2;
                        return false;
                    }
                } else {
                    ROS_DEBUG_NAMED(SCM_LOGGER, "  *collision* '%s' x '%s'", s1m->name.c_str(), s2m->name.c_str());
                    dist = cd2;
                    return false;
                }
            }
            // collision between leaves is ok
            continue;
        }

        // choose a sphere node to split
        bool split1;
        if (s1s->isLeaf()) { // split sphere 2
            split1 = false;
        } else if (s2s->isLeaf()) { // split sphere 1
            split1 = true;
        } else { // choose a node to split
            // heuristic -> split the larger sphere to obtain more
            // information about the underlying surface, assuming the leaf
            // spheres are often about the same size
            if (s1m->radius > s2m->radius) {
                split1 = true;
            } else {
                split1 = false;
            }
        }

        if (split1) {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Splitting node '%s'", s1m->name.c_str());
            const CollisionSphereState* sl = s1s->left;
            const CollisionSphereState* sr = s1s->right;
            // update children positions
            stateA.updateSphereState(SphereIndex(ss1i, sl->index()));
            stateA.updateSphereState(SphereIndex(ss1i, sr->index()));

            // heuristic -> examine the pair of spheres that are closer together
            // first for a better chance at detecting collision

            // dist from split sphere's left child to other sphere
            const double cd2l2 = (s2s->pos - sl->pos).squaredNorm();
            // dist from split sphere's right child to other sphere
            const double cd2r2 = (s2s->pos - sr->pos).squaredNorm();

            if (cd2l2 < cd2r2) {
                // examine right child after the left child
                q.push_back(std::make_pair(sr, s2s));
                q.push_back(std::make_pair(sl, s2s));
            } else {
                // examine the left child after the right child
                q.push_back(std::make_pair(sl, s2s));
                q.push_back(std::make_pair(sr, s2s));
            }
        } else {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Splitting node '%s'", s2m->name.c_str());
            // equivalent comments from above
            const CollisionSphereState* sl = s2s->left;
            const CollisionSphereState* sr = s2s->right;

            stateB.updateSphereState(SphereIndex(ss2i, sl->index()));
            stateB.updateSphereState(SphereIndex(ss2i, sr->index()));

            double cd1l2 = (s1s->pos - sl->pos).squaredNorm();
            double cd1r2 = (s1s->pos - sr->pos).squaredNorm();

            if (cd1l2 < cd1r2) {
                q.push_back(std::make_pair(s1s, sr));
                q.push_back(std::make_pair(s1s, sl));
            } else {
                q.push_back(std::make_pair(s1s, sl));
                q.push_back(std::make_pair(s1s, sr));
            }
        }
    }
    ROS_DEBUG_NAMED(SCM_LOGGER, "queue exhaused");

    // queue exhaused = no collision found
    return true;
}

void SelfCollisionModel::updateCheckedSpheresIndices()
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Update checked sphere indices");

    if (m_gidx == -1) {
        return;
    }

    updateRobotCheckedSphereIndices();
    updateAttachedBodyCheckedSphereIndices();
    updateRobotAttachedBodyCheckedSphereIndices();
}

void SelfCollisionModel::updateRobotCheckedSphereIndices()
{
    m_checked_spheres_states.clear();

    const auto& group_link_indices = m_rcm->groupLinkIndices(m_gidx);
    for (int l1 = 0; l1 < group_link_indices.size(); ++l1) {
        const int lidx1 = group_link_indices[l1];
        const bool l1_has_spheres = m_rcm->hasSpheresModel(lidx1);
        if (!l1_has_spheres) {
            continue;
        }
        const std::string& l1_name = m_rcm->linkName(lidx1);
        for (int l2 = l1 + 1; l2 < group_link_indices.size(); ++l2) {
            const int lidx2 = group_link_indices[l2];
            const bool l2_has_spheres = m_rcm->hasSpheresModel(lidx2);
            if (!l2_has_spheres) {
                continue;
            }
            const std::string& l2_name = m_rcm->linkName(lidx2);

            collision_detection::AllowedCollision::Type type;
            if (m_acm.getEntry(l1_name, l2_name, type)) {
                if (type != collision_detection::AllowedCollision::ALWAYS) {
                    m_checked_spheres_states.emplace_back(
                            m_rcs.linkSpheresStateIndex(lidx1),
                            m_rcs.linkSpheresStateIndex(lidx2));
                }
            }
            else {
                m_checked_spheres_states.emplace_back(
                        m_rcs.linkSpheresStateIndex(lidx1),
                        m_rcs.linkSpheresStateIndex(lidx2));
            }
        }
    }
}

void SelfCollisionModel::updateRobotAttachedBodyCheckedSphereIndices()
{
    // TODO: see note in updateAttachedBodyCheckedSphereIndices()
    m_checked_attached_body_robot_spheres_states.clear();

    const auto& group_body_indices = m_abcm->groupLinkIndices(m_gidx);
    const auto& group_link_indices = m_rcm->groupLinkIndices(m_gidx);
    for (int b1 = 0; b1 < group_body_indices.size(); ++b1) {
        const int bidx1 = group_body_indices[b1];
        const bool b1_has_spheres = m_abcm->hasSpheresModel(bidx1);
        if (!b1_has_spheres) {
            continue;
        }
        const std::string& b1_name = m_abcm->attachedBodyName(bidx1);
        for (int l1 = 0; l1 < group_link_indices.size(); ++l1) {
            const int lidx = group_link_indices[l1];
            const bool l1_has_spheres = m_rcm->hasSpheresModel(lidx);
            if (!l1_has_spheres) {
                continue;
            }
            const std::string& l1_name = m_rcm->linkName(lidx);

            collision_detection::AllowedCollision::Type type;
            if (m_acm.getEntry(b1_name, l1_name, type)) {
                if (type != collision_detection::AllowedCollision::ALWAYS) {
                    m_checked_attached_body_robot_spheres_states.emplace_back(
                            m_abcs.attachedBodySpheresStateIndex(bidx1),
                            m_rcs.linkSpheresStateIndex(lidx));
                }
            }
            else {
                m_checked_attached_body_robot_spheres_states.emplace_back(
                        m_abcs.attachedBodySpheresStateIndex(bidx1),
                        m_rcs.linkSpheresStateIndex(lidx));
            }
        }
    }
}

void SelfCollisionModel::updateAttachedBodyCheckedSphereIndices()
{
    // TODO: need to call this function whenever the attached bodies model has
    // a body added or removed from it
    m_checked_attached_body_spheres_states.clear();
    const auto& group_body_indices = m_abcm->groupLinkIndices(m_gidx);
    for (int b1 = 0; b1 < group_body_indices.size(); ++b1) {
        const int bidx1 = group_body_indices[b1];
        bool b1_has_spheres = m_abcm->hasSpheresModel(bidx1);
        if (!b1_has_spheres) {
            continue;
        }
        const std::string& b1_name = m_abcm->attachedBodyName(bidx1);
        for (int b2 = b1 + 1; b2 < group_body_indices.size(); ++b2) {
            const int bidx2 = group_body_indices[b2];
            const bool b2_has_spheres = m_abcm->hasSpheresModel(bidx2);
            if (!b2_has_spheres) {
                continue;
            }
            const std::string& b2_name = m_abcm->attachedBodyName(bidx2);

            collision_detection::AllowedCollision::Type type;
            if (m_acm.getEntry(b1_name, b2_name, type)) {
                if (type != collision_detection::AllowedCollision::ALWAYS) {
                    m_checked_attached_body_spheres_states.emplace_back(
                            m_abcs.attachedBodySpheresStateIndex(bidx1),
                            m_abcs.attachedBodySpheresStateIndex(bidx2));
                }
            }
            else {
                m_checked_attached_body_spheres_states.emplace_back(
                        m_abcs.attachedBodySpheresStateIndex(bidx1),
                        m_abcs.attachedBodySpheresStateIndex(bidx2));
            }
        }
    }
}

void SelfCollisionModel::updateMetaSphereTrees()
{
#if SCDL_USE_META_TREE
    // update the root collision sphere models
    const auto& spheres_state_indices = m_rcs.groupSpheresStateIndices(m_gidx);
    for (size_t i = 0; i < spheres_state_indices.size(); ++i) {
        const int ssidx = spheres_state_indices[i];
        // update the model with the position of the root spheres state
        const auto& ss = m_rcs.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        m_rcs.updateSphereState(SphereIndex(ssidx, s->index()));
        CollisionSphereModel& sm = m_root_models[i];
        sm.center = s->pos;
    }

    // rebuild the meta tree model
    m_meta_model.spheres.buildFrom(m_root_model_pointers);
    for (auto& sphere : m_meta_model.spheres.m_tree) {
        sphere.parent = &m_meta_model;
    }

    // recreate the meta tree state
    m_meta_state.spheres.buildFrom(&m_meta_state);

    // meta-leaf spheres will have null children at this point
    // -> update meta-leaf sphere states' children to be the root states of
    //    other subtrees
    for (auto& ss : m_meta_state.spheres) {
        if (ss.isLeaf()) {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Mapping s.left (%p) to (%p)", ss.left, m_model_state_map[ss.model->left]);
            ROS_DEBUG_NAMED(SCM_LOGGER, "Mapping s.right (%p) to (%p)", ss.right, m_model_state_map[ss.model->right]);
            // model.leaf points to the correct child
            ss.left = m_model_state_map[ss.model->left];
            ss.right = m_model_state_map[ss.model->right];
        }
    }
#endif
}

double SelfCollisionModel::robotVoxelsCollisionDistance()
{
    auto& q = m_vq;
    q.clear();

    for (const int ssidx : m_rcs.groupSpheresStateIndices(m_gidx)) {
        const auto& ss = m_rcs.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        q.push_back(s);
    }

    double d = std::numeric_limits<double>::infinity();

    while (!q.empty()) {
        const CollisionSphereState* s = q.back();
        q.pop_back();

        // update non-meta states
        if (s->parent_state->index != -1) {
            m_rcs.updateSphereState(SphereIndex(s->parent_state->index, s->index()));
        }

        ROS_DEBUG_NAMED(SCM_LOGGER, "Checking sphere with radius %0.3f at (%0.3f, %0.3f, %0.3f)", s->model->radius, s->pos.x(), s->pos.y(), s->pos.z());

        double obs_dist = SphereCollisionDistance(*m_grid, *s, m_padding);
        if (obs_dist >= d) {
            continue; // further -> ok!
        }

        const double alpha = 0.5;
        d = std::max(0.0, (1.0 - alpha) * obs_dist);
        if (d == 0.0) {
            // can't lower separation distance further -> done!
            q.clear();
            continue;
        }

        // collision -> not ok or recurse!

        if (s->isLeaf()) {
            if (s->parent_state->index == -1) {
                // meta-leaf -> recurse on existing children of referenced
                // sphere tree root state

                // node connecting meta tree to kinematic tree
                assert(s->left == s->right);
                const CollisionSphereState* sl = s->left->left;
                const CollisionSphereState* sr = s->right->right;

                if (sl && sr) {
                    if (sl->model->radius > sr->model->radius) {
                        q.push_back(sr);
                        q.push_back(sl);
                    }
                    else {
                        q.push_back(sl);
                        q.push_back(sr);
                    }
                }
                else if (sl) {
                    q.push_back(sl);
                }
                else if (sr) {
                    q.push_back(sr);
                }
            }
            // else continue checking other subtrees
        }
        else { // recurse on both the children
            if (s->left->model->radius > s->right->model->radius) {
                q.push_back(s->right);
                q.push_back(s->left);
            }
            else {
                q.push_back(s->left);
                q.push_back(s->right);
            }
        }
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "voxels distance = %0.3f", d);
    return d;
}

double SelfCollisionModel::robotSpheresCollisionDistance()
{
    double dmin = std::numeric_limits<double>::infinity();
    for (const auto& ss_pair: m_checked_spheres_states) {
        int ss1i = ss_pair.first;
        int ss2i = ss_pair.second;
        const CollisionSpheresState& ss1 = m_rcs.spheresState(ss1i);
        const CollisionSpheresState& ss2 = m_rcs.spheresState(ss2i);

        double d = spheresStateCollisionDistance(ss1i, ss2i, ss1, ss2);
        if (d < dmin) {
            dmin = d;
        }
    }
    return dmin;
}

double SelfCollisionModel::robotSpheresCollisionDistance(
    const AllowedCollisionsInterface& aci)
{
    // TODO: implement
    return std::numeric_limits<double>::infinity();
}

double SelfCollisionModel::attachedBodyVoxelsCollisionDistance()
{
    // TODO: implement
    return std::numeric_limits<double>::infinity();
}

double SelfCollisionModel::attachedBodySpheresCollisionDistance()
{
    // TODO: implement
    return std::numeric_limits<double>::infinity();
}

double SelfCollisionModel::attachedBodySpheresCollisionDistance(
    const AllowedCollisionsInterface& aci)
{
    // TODO: implement
    return std::numeric_limits<double>::infinity();
}

double SelfCollisionModel::spheresStateCollisionDistance(
    const int ss1i, const int ss2i,
    const CollisionSpheresState& ss1,
    const CollisionSpheresState& ss2)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check spheres state collision");

    double dp = std::numeric_limits<double>::infinity();

    // assertion: both collision spheres are updated when they are removed from the stack
    m_rcs.updateSphereState(SphereIndex(ss1i, ss1.spheres.root()->index()));
    m_rcs.updateSphereState(SphereIndex(ss2i, ss2.spheres.root()->index()));

    auto& q = m_q;
    q.clear();
    q.push_back(std::make_pair(ss1.spheres.root(), ss2.spheres.root()));
    while (!q.empty()) {
        // get the next pair of spheres to check
        const CollisionSphereState *s1s, *s2s;
        std::tie(s1s, s2s) = q.back();
        q.pop_back();

        const CollisionSphereModel* s1m = s1s->model;
        const CollisionSphereModel* s2m = s2s->model;

        ROS_DEBUG_NAMED(SCM_LOGGER, "Checking '%s' x '%s' collision", s1m->name.c_str(), s2m->name.c_str());

        // NOTE: this algorithm doesn't play nicely with the concept of allowed
        // collisions between individual spheres. The assumption is that the
        // current estimate is an upper bound on the minimum separate distance
        // but, in the case of an allowed collision matrix, we may find out
        // further down the tree that the upper bound can be much greater when
        // ignoring checks between individual spheres. (which return inf as the
        // bound on their separation distance)

        double ds = sphereDistance(*s1s, *s2s);
        if (ds >= dp) {
            continue; // collision is greater -> back out
        }

        // TODO: does it make sense to keep negative distances around to serve
        // as a penetration distance metric?
        const double alpha = 0.5; // technically, (1 - alpha) from the paper
        dp = std::max(0.0, (1.0 - alpha) * ds);

        if (dp == 0) {
            // can't possibly lower the separate distance further -> done!
            q.clear();
            continue;
        }

        if (s1s->isLeaf() && s2s->isLeaf()) {
            // done recursing
            continue;
        }

        // choose a sphere node to split
        bool split1;
        if (s1s->isLeaf()) { // split sphere 2
            split1 = false;
        }
        else if (s2s->isLeaf()) { // split sphere 1
            split1 = true;
        }
        else { // choose a node to split
            // heuristic -> split the larger sphere to obtain more
            // information about the underlying surface, assuming the leaf
            // spheres are often about the same size
            if (s1m->radius > s2m->radius) {
                split1 = true;
            }
            else {
                split1 = false;
            }
        }

        if (split1) {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Splitting node '%s'", s1m->name.c_str());
            const CollisionSphereState* sl = s1s->left;
            const CollisionSphereState* sr = s1s->right;
            // update children positions
            m_rcs.updateSphereState(SphereIndex(ss1i, sl->index()));
            m_rcs.updateSphereState(SphereIndex(ss1i, sr->index()));

            // heuristic -> examine the pair of spheres that are closer together
            // first for a better chance at detecting collision

            // dist from split sphere's left child to other sphere
            const double cd2l2 = (s2s->pos - sl->pos).squaredNorm();
            // dist from split sphere's right child to other sphere
            const double cd2r2 = (s2s->pos - sr->pos).squaredNorm();

            if (cd2l2 < cd2r2) {
                // examine right child after the left child
                q.push_back(std::make_pair(sr, s2s));
                q.push_back(std::make_pair(sl, s2s));
            }
            else {
                // examine the left child after the right child
                q.push_back(std::make_pair(sl, s2s));
                q.push_back(std::make_pair(sr, s2s));
            }
        }
        else {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Splitting node '%s'", s2m->name.c_str());
            // equivalent comments from above
            const CollisionSphereState* sl = s2s->left;
            const CollisionSphereState* sr = s2s->right;

            m_rcs.updateSphereState(SphereIndex(ss2i, sl->index()));
            m_rcs.updateSphereState(SphereIndex(ss2i, sr->index()));

            double cd1l2 = (s1s->pos - sl->pos).squaredNorm();
            double cd1r2 = (s1s->pos - sr->pos).squaredNorm();

            if (cd1l2 < cd1r2) {
                q.push_back(std::make_pair(s1s, sr));
                q.push_back(std::make_pair(s1s, sl));
            }
            else {
                q.push_back(std::make_pair(s1s, sl));
                q.push_back(std::make_pair(s1s, sr));
            }
        }
    }
    ROS_DEBUG_NAMED(SCM_LOGGER, "queue exhaused");

    // queue exhaused = no collision found
    return true;
}

double SelfCollisionModel::sphereDistance(
    const CollisionSphereState& s1,
    const CollisionSphereState& s2) const
{
    return (s2.pos - s1.pos).norm() - s1.model->radius - s2.model->radius;
}

bool SelfCollisionModel::getRobotVoxelsStateCollisionDetails(
    CollisionDetails& details)
{
    const size_t old_size = details.details.size();

    for (const int ssidx : m_rcs.groupSpheresStateIndices(m_gidx)) {
        auto& q = m_vq;
        q.clear();

        const auto& ss = m_rcs.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        q.push_back(s);

        double dist;
        if (!CheckVoxelsCollisions(m_rcs, q, *m_grid, m_padding, dist)) {
            CollisionDetail detail;
            detail.first_link = m_rcs.model()->linkName(ss.model->link_index);
            detail.second_link = "_voxels_";
            detail.penetration = dist;
            details.details.push_back(detail);
        }
    }

    return details.details.size() == old_size;
}

bool SelfCollisionModel::getAttachedBodyVoxelsStateCollisionDetails(
    CollisionDetails& details)
{
    double dist;
    if (!checkAttachedBodyVoxelsStateCollisions(dist)) {
        ++details.voxels_collision_count;
        return false;
    }
    return true;
}

bool SelfCollisionModel::getRobotSpheresStateCollisionDetails(
    CollisionDetails& details)
{
    size_t old_size = details.details.size();

    ROS_DEBUG_NAMED(SCM_LOGGER, "Check robot links vs robot links");

    for (const auto& ss_pair : m_checked_spheres_states) {
        int ss1i = ss_pair.first;
        int ss2i = ss_pair.second;
        const CollisionSpheresState& ss1 = m_rcs.spheresState(ss1i);
        const CollisionSpheresState& ss2 = m_rcs.spheresState(ss2i);

        double dist;
        if (!checkSpheresStateCollision(
                m_rcs, m_rcs, ss1i, ss2i, ss1, ss2, dist))
        {
            CollisionDetail detail;
            detail.first_link = m_rcs.model()->linkName(ss1.model->link_index);
            detail.second_link = m_rcs.model()->linkName(ss2.model->link_index);
            detail.penetration = dist;
            details.details.push_back(detail);
        }
    }

    details.sphere_collision_count = details.details.size() - old_size;
    return details.details.size() == old_size;
}

bool SelfCollisionModel::getRobotSpheresStateCollisionDetails(
    const AllowedCollisionsInterface& aci,
    CollisionDetails& details)
{
    double dist;
    if (!checkRobotSpheresStateCollisions(aci, dist)) {
        ++details.sphere_collision_count;
        return false;
    }
    return true;
}

bool SelfCollisionModel::getAttachedBodySpheresStateCollisionDetails(
    CollisionDetails& details)
{
    double dist;
    if (!checkAttachedBodySpheresStateCollisions(dist)) {
        ++details.sphere_collision_count;
        return false;
    }
    return true;
}

bool SelfCollisionModel::getAttachedBodySpheresStateCollisionDetails(
    const AllowedCollisionsInterface& aci,
    CollisionDetails& details)
{
    double dist;
    if (!checkAttachedBodySpheresStateCollisions(aci, dist)) {
        ++details.sphere_collision_count;
        return false;
    }
    return true;
}

bool SelfCollisionModel::getRobotAttachedBodySpheresStateCollisionDetails(
    CollisionDetails& details)
{
    return false;
}

bool SelfCollisionModel::getRobotAttachedBodySpheresStateCollisionDetails(
    const AllowedCollisionsInterface& aci,
    CollisionDetails& details)
{
    return false;
}

} // namespace collision
} // namespace smpl
