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

#ifndef sbpl_collision_attached_bodies_collision_model_h
#define sbpl_collision_attached_bodies_collision_model_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <geometric_shapes/shapes.h>

// project includes
#include <sbpl_collision_checking/base_collision_models.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/types.h>

namespace smpl {
namespace collision {

class AttachedBodiesCollisionModel
{
public:

    AttachedBodiesCollisionModel(const RobotCollisionModel* model);

    /// \name Attached Bodies Model
    ///@{
    bool attachBody(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Isometry3dVector& transforms,
        const std::string& link_name,
        bool create_voxels_model = true,
        bool create_spheres_model = true);
    bool detachBody(const std::string& id);

    size_t attachedBodyCount() const;
    bool   hasAttachedBody(const std::string& id) const;
    int    attachedBodyIndex(const std::string& id) const;
    auto   attachedBodyName(int abidx) const -> const std::string&;
    int    attachedBodyLinkIndex(int abidx) const;

    void   attachedBodyIndices(std::vector<int>& indices) const;

    auto attachedBodyIndices(const std::string& link_name) const
            -> const std::vector<int>&;
    auto attachedBodyIndices(int lidx) const -> const std::vector<int>&;

    int version() const;
    ///@}

    /// \name Attached Bodies Collision Model
    ///@{
    size_t sphereModelCount() const;

    bool   hasSpheresModel(const std::string& id) const;
    bool   hasSpheresModel(int abidx) const;
    size_t spheresModelCount() const;
    auto   spheresModel(int smidx) const -> const CollisionSpheresModel&;

    bool   hasVoxelsModel(const std::string& id) const;
    bool   hasVoxelsModel(int abidx) const;
    size_t voxelsModelCount() const;
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    size_t groupCount() const;
    auto   group(int gidx) const -> const CollisionGroupModel&;
    bool   hasGroup(const std::string& group_name) const;
    int    groupIndex(const std::string& group_name) const;
    auto   groupName(int gidx) const -> const std::string&;

    auto groupLinkIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto groupLinkIndices(int gidx) const -> const std::vector<int>&;
    ///@}

private:

    struct AttachedBodyModel
    {
        std::string id;
        int link_index;

        CollisionSpheresModel* spheres_model;
        CollisionVoxelsModel* voxels_model;
    };

    const RobotCollisionModel*                  m_model;

    // set of attached bodies
    hash_map<int, AttachedBodyModel>            m_attached_bodies;

    // map attached body names to attached body indices
    hash_map<std::string, int>                  m_attached_body_name_to_index;

    int                                         m_attached_bodies_added;

    // map link indices to attached body indices
    std::vector<std::vector<int>>               m_link_attached_bodies;

    // Collision Model
    std::vector<std::unique_ptr<CollisionSpheresModel>> m_spheres_models;
    std::vector<std::unique_ptr<CollisionVoxelsModel>>  m_voxels_models;
    std::vector<CollisionGroupModel>                    m_group_models;
    hash_map<std::string, int>                          m_group_name_to_index;

    int m_version;

    int generateAttachedBodyIndex();

    CollisionSpheresModel* createSpheresModel(
        int abidx,
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Isometry3dVector& transforms);

    CollisionVoxelsModel* createVoxelsModel(
        int abidx,
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Isometry3dVector& transforms);

    void generateSpheresModel(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Isometry3dVector& transforms,
        CollisionSpheresModelConfig& spheres_model);

    void generateVoxelsModel(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Isometry3dVector& transforms,
        CollisionVoxelModelConfig& voxels_models);

    bool voxelizeAttachedBody(
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Isometry3dVector& transforms,
        CollisionVoxelsModel& model) const;
};

typedef std::shared_ptr<AttachedBodiesCollisionModel> AttachedBodiesCollisionModelPtr;
typedef std::shared_ptr<const AttachedBodiesCollisionModel> AttachedBodiesCollisionModelConstPtr;

inline
size_t AttachedBodiesCollisionModel::attachedBodyCount() const
{
    return m_attached_bodies.size();
}

inline
bool AttachedBodiesCollisionModel::hasAttachedBody(
    const std::string& id) const
{
    return m_attached_body_name_to_index.find(id) !=
            m_attached_body_name_to_index.end();
}

inline
int AttachedBodiesCollisionModel::attachedBodyIndex(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    assert(m_attached_bodies.find(it->second) != m_attached_bodies.end());
    return it->second;
}

inline
const std::string& AttachedBodiesCollisionModel::attachedBodyName(
    int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.id;
}

inline
int AttachedBodiesCollisionModel::attachedBodyLinkIndex(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.link_index;
}

inline
const std::vector<int>& AttachedBodiesCollisionModel::attachedBodyIndices(
    const std::string& link_name) const
{
    const int lidx = m_model->linkIndex(link_name);
    return m_link_attached_bodies[lidx];
}

inline
void AttachedBodiesCollisionModel::attachedBodyIndices(
    std::vector<int>& indices) const
{
    for (const auto& entry : m_attached_bodies) {
        indices.push_back(entry.first);
    }
}

inline
const std::vector<int>& AttachedBodiesCollisionModel::attachedBodyIndices(
    int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_attached_bodies, lidx);
    return m_link_attached_bodies[lidx];
}

inline
int AttachedBodiesCollisionModel::version() const
{
    return m_version;
}

inline
size_t AttachedBodiesCollisionModel::sphereModelCount() const
{
    size_t total_sphere_count = 0;
    for (const auto& spheres_model : m_spheres_models) {
        total_sphere_count += spheres_model->spheres.size();
    }
    return total_sphere_count;
}

inline
bool AttachedBodiesCollisionModel::hasSpheresModel(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    auto abit = m_attached_bodies.find(it->second);
    assert(abit != m_attached_bodies.end());
    return abit->second.spheres_model;
}

inline
bool AttachedBodiesCollisionModel::hasSpheresModel(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.spheres_model;
}

inline
size_t AttachedBodiesCollisionModel::spheresModelCount() const
{
    return m_spheres_models.size();
}

inline
const CollisionSpheresModel& AttachedBodiesCollisionModel::spheresModel(
    int smidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_models, smidx);
    return *m_spheres_models[smidx];
}

inline
bool AttachedBodiesCollisionModel::hasVoxelsModel(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    auto abit = m_attached_bodies.find(it->second);
    assert(abit != m_attached_bodies.end());
    return abit->second.voxels_model;
}

inline
bool AttachedBodiesCollisionModel::hasVoxelsModel(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.voxels_model;
}

inline
size_t AttachedBodiesCollisionModel::voxelsModelCount() const
{
    return m_voxels_models.size();
}

inline
const CollisionVoxelsModel& AttachedBodiesCollisionModel::voxelsModel(
    int vmidx) const
{
    ASSERT_VECTOR_RANGE(m_voxels_models, vmidx);
    return *m_voxels_models[vmidx];
}

inline
size_t AttachedBodiesCollisionModel::groupCount() const
{
    return m_group_models.size();
}

inline
const CollisionGroupModel& AttachedBodiesCollisionModel::group(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx];
}

inline
bool AttachedBodiesCollisionModel::hasGroup(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    return it != m_group_name_to_index.end();
}

inline
int AttachedBodiesCollisionModel::groupIndex(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    return it->second;
}

inline
const std::string& AttachedBodiesCollisionModel::groupName(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].name;
}

inline
const std::vector<int>& AttachedBodiesCollisionModel::groupLinkIndices(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    return m_group_models[it->second].link_indices;
}

inline
const std::vector<int>& AttachedBodiesCollisionModel::groupLinkIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].link_indices;
}

} // namespace collision
} // namespace smpl

#endif
