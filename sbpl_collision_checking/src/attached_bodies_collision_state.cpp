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

#include <algorithm>

#include <sbpl_collision_checking/attached_bodies_collision_state.h>
#include <sbpl_collision_checking/debug.h>

namespace smpl {
namespace collision {

static const char* ABS_LOGGER = "attached_bodies_state";

AttachedBodiesCollisionState::AttachedBodiesCollisionState(
    const AttachedBodiesCollisionModel* model,
    RobotCollisionState* state)
:
    m_model(model),
    m_state(state),
    m_voxels_state_versions(),
    m_voxels_states(),
    m_group_states(),
    m_version(-1) // -1 to force first initial reinitialization
{
}

bool AttachedBodiesCollisionState::updateVoxelsState(int vsidx)
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_voxels_state_versions, vsidx);

    if (!voxelsStateDirty(vsidx)) {
        return false;
    }

    CollisionVoxelsState& state = m_voxels_states[vsidx];

    const int bidx = state.model->link_index;
    updateAttachedBodyTransform(bidx);

    const Eigen::Affine3d& T_model_body = attachedBodyTransform(bidx);

    // transform voxels into the model frame
    std::vector<Eigen::Vector3d> new_voxels(state.model->voxels.size());
    for (size_t i = 0; i < state.model->voxels.size(); ++i) {
        new_voxels[i] = T_model_body * state.model->voxels[i];
    }

    state.voxels = std::move(new_voxels);
    
    if (m_voxels_state_versions.size()<=vsidx)
        m_voxels_state_versions.resize(vsidx+1);

    m_voxels_state_versions[vsidx] = attachedBodyTransformVersion(bidx);
    return true;
}

bool AttachedBodiesCollisionState::updateSphereState(
    const SphereIndex& sidx)
{
    reinitCollisionState();

    CollisionSpheresState& spheres_state = m_spheres_states[sidx.ss];
    const int bidx = spheres_state.model->link_index;
    const int body_version = attachedBodyTransformVersion(bidx);
    CollisionSphereState& sphere_state = spheres_state.spheres[sidx.s];

    if (!attachedBodyTransformDirty(bidx) &&
        sphere_state.version == body_version)
    {
        return false;
    }

    updateAttachedBodyTransform(bidx);

    ROS_DEBUG_NAMED(ABS_LOGGER, "Updating position of sphere '%s'", sphere_state.model->name.c_str());
    const Eigen::Affine3d& T_model_body = attachedBodyTransform(bidx);
    sphere_state.pos = T_model_body * sphere_state.model->center;

    sphere_state.version = attachedBodyTransformVersion(bidx);
    return true;
}

void AttachedBodiesCollisionState::reinitCollisionState()
{
    if (m_version == m_model->version()) {
        return;
    }

    ROS_DEBUG_NAMED(ABS_LOGGER, "Update Attached Bodies Collision State to version %d", m_model->version());

    // initialize spheres states
    m_spheres_states.assign(
            m_model->spheresModelCount(), CollisionSpheresState());
    for (size_t i = 0; i < m_model->spheresModelCount(); ++i) {
        CollisionSpheresState& spheres_state = m_spheres_states[i];
        spheres_state.model = &m_model->spheresModel(i);
        spheres_state.spheres.buildFrom(&spheres_state);
    }

    // initialize voxels states
    m_voxels_states.assign(m_model->voxelsModelCount(), CollisionVoxelsState());

    for (size_t i = 0; i < m_model->voxelsModelCount(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
        CollisionVoxelsState& voxels_state = m_voxels_states[i];
        // duplicate voxels from voxels model
        voxels_state.model = &voxels_model;
        voxels_state.voxels = voxels_model.voxels;
    }

    // initialize group states
    m_group_states.assign(m_model->groupCount(), CollisionGroupState());
    for (size_t i = 0; i < m_model->groupCount(); ++i) {
        const CollisionGroupModel& group_model = m_model->group(i);
        CollisionGroupState& group_state = m_group_states[i];
        // map group state -> group model
        group_state.model = &group_model;
    }

    // map group state -> spheres states
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        // get the link this spheres state is attached to
        const int lidx = spheres_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) !=
                group_model.link_indices.end())
            {
                m_group_states[gidx].spheres_indices.push_back(ssidx);
            }
        }
    }

    // map group state -> (outside) voxels states
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        const CollisionVoxelsState& voxels_state = m_voxels_states[vsidx];
        // get the link this spheres state is attached to
        const int lidx = voxels_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) ==
                group_model.link_indices.end())
            {
                m_group_states[gidx].voxels_indices.push_back(vsidx);
            }
        }
    }

    m_attached_bodies_states.clear();
    for (CollisionVoxelsState& vs : m_voxels_states) {
        int abidx = vs.model->link_index;
        m_attached_bodies_states[abidx].voxels_state = &vs;
    }
    for (CollisionSpheresState& ss : m_spheres_states) {
        int abidx = ss.model->link_index;
        m_attached_bodies_states[abidx].spheres_state = &ss;
    }

    m_version = m_model->version();
}

} // namespace collision
} // namespace smpl
