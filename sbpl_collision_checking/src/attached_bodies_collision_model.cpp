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

// project includes
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/debug.h>
#include <sbpl_collision_checking/voxel_operations.h>

namespace smpl {
namespace collision {

static const char* ABM_LOGGER = "attached_bodies_model";

AttachedBodiesCollisionModel::AttachedBodiesCollisionModel(
    const RobotCollisionModel* model)
:
    m_model(model),
    m_attached_bodies(),
    m_attached_body_name_to_index(),
    m_attached_bodies_added(),
    m_link_attached_bodies(),
    m_spheres_models(),
    m_voxels_models(),
    m_group_models(),
    m_group_name_to_index(),
    m_version(0)
{
    m_link_attached_bodies.resize(m_model->linkCount());

    // use group names identical to robot model
    m_group_models.resize(model->groupCount());
    for (int gidx = 0; gidx < model->groupCount(); ++gidx) {
        m_group_models[gidx].name = model->group(gidx).name;
        m_group_name_to_index[model->group(gidx).name] = gidx;
    }
}

/// \brief Attach a body to the collision model
/// \param shapes The shapes making up the body
/// \param transforms The offsets from the attached link for each shape
/// \param link_name The link to attach to
bool AttachedBodiesCollisionModel::attachBody(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Isometry3dVector& transforms,
    const std::string& link_name,
    bool create_voxels_model,
    bool create_spheres_model)
{
    if (hasAttachedBody(id)) {
        ROS_WARN_NAMED(ABM_LOGGER, "Already have attached body '%s'", id.c_str());
        return false;
    }

    if (!m_model->hasLink(link_name)) {
        ROS_WARN_NAMED(ABM_LOGGER, "Link '%s' does not exist in the Robot Collision Model", link_name.c_str());
        return false;
    }

    ROS_DEBUG_NAMED(ABM_LOGGER, "Attach body '%s'", id.c_str());

    const int abidx = generateAttachedBodyIndex();
    ROS_DEBUG_NAMED(ABM_LOGGER, "  Body Index: %d", abidx);

    // create the attached body descriptor
    AttachedBodyModel ab;
    ab.id = id;
    ab.link_index = m_model->linkIndex(link_name);

    if (create_spheres_model) {
        ab.spheres_model = createSpheresModel(abidx, id, shapes, transforms);
    } else {
        ab.spheres_model = nullptr;
    }

    if (create_voxels_model) {
        ab.voxels_model = createVoxelsModel(abidx, id, shapes, transforms);
    } else {
        ab.voxels_model = nullptr;
    }

    ROS_DEBUG_NAMED(ABM_LOGGER, " -> Link Index: %d", ab.link_index);
    ROS_DEBUG_NAMED(ABM_LOGGER, " -> Spheres Model: %p", ab.spheres_model);
    ROS_DEBUG_NAMED(ABM_LOGGER, " -> Voxels Model: %p", ab.voxels_model);

    m_attached_bodies[abidx] = std::move(ab);

    // map the id to the generated index
    m_attached_body_name_to_index[id] = abidx;

    // attach the body to its parent link
    m_link_attached_bodies[ab.link_index].push_back(abidx);

    ROS_DEBUG_NAMED(ABM_LOGGER, "Update group model");

    // add attached body to all groups whose corresponding robot model groups
    // contain its parent link
    for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
        const CollisionGroupModel& parent_group_model = m_model->group(gidx);
        if (std::find(
                parent_group_model.link_indices.begin(),
                parent_group_model.link_indices.end(),
                ab.link_index) !=
            parent_group_model.link_indices.end())
        {
            ROS_DEBUG_NAMED(ABM_LOGGER, "Add attached body to group '%s'", m_model->groupName(gidx).c_str());
            m_group_models[gidx].link_indices.push_back(abidx);
        }
    }

    ++m_version;
    return true;
}

bool AttachedBodiesCollisionModel::detachBody(const std::string& id)
{
    if (!hasAttachedBody(id)) {
        ROS_WARN_NAMED(ABM_LOGGER, "No attached body '%s' found in Robot Collision Model", id.c_str());
        return false;
    }

    int abidx = attachedBodyIndex(id);

    // remove from group models
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        CollisionGroupModel& group_model = m_group_models[i];
        auto it = std::remove_if(
                group_model.link_indices.begin(),
                group_model.link_indices.end(),
                [&](int bidx) { return bidx == abidx; });
        group_model.link_indices.erase(it, group_model.link_indices.end());
    }

    auto abit = m_attached_bodies.find(abidx);

    if (abit->second.voxels_model) {
        auto it = std::remove_if(m_voxels_models.begin(), m_voxels_models.end(),
                [&](const std::unique_ptr<CollisionVoxelsModel>& vm)
                {
                    return vm.get() == abit->second.voxels_model;
                });
        m_voxels_models.erase(it, m_voxels_models.end());
    }

    if (abit->second.spheres_model) {
        auto it = std::remove_if(m_spheres_models.begin(), m_spheres_models.end(),
                [&](const std::unique_ptr<CollisionSpheresModel>& sm)
                {
                    return sm.get() == abit->second.spheres_model;
                });
        m_spheres_models.erase(it, m_spheres_models.end());
    }

    // remove the attached body from its attached link
    std::vector<int>& link_bodies =
            m_link_attached_bodies[abit->second.link_index];
    auto it = std::remove_if(link_bodies.begin(), link_bodies.end(),
            [abidx](int idx) { return idx == abidx; });
    link_bodies.erase(it, link_bodies.end());

    int ret;

    // remove the id -> index mapping
    ret = m_attached_body_name_to_index.erase(id);
    assert(ret > 0);

    // remove the attached body
    m_attached_bodies.erase(abit);

    ++m_version;
    return true;
}

CollisionSpheresModel* AttachedBodiesCollisionModel::createSpheresModel(
    int abidx,
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Isometry3dVector& transforms)
{
    ROS_DEBUG_NAMED(ABM_LOGGER, "  Generate spheres model");

    // create configuration spheres and a spheres model for this body
    CollisionSpheresModelConfig config;
    generateSpheresModel(id, shapes, transforms, config);

    // initialize a new spheres model
    m_spheres_models.emplace_back(new CollisionSpheresModel);
    ROS_DEBUG_NAMED(ABM_LOGGER, "  Spheres Model Count %zu", m_spheres_models.size());

    CollisionSpheresModel* spheres_model = m_spheres_models.back().get();

    spheres_model->link_index = abidx;
    spheres_model->spheres.buildFrom(config.spheres);
    ROS_DEBUG_NAMED(ABM_LOGGER, "  Spheres Model: %p", spheres_model);

    // TODO: possible make this more automatic?
    for (auto& sphere : spheres_model->spheres.m_tree) {
        sphere.parent = spheres_model;
    }

    return spheres_model;
}

CollisionVoxelsModel* AttachedBodiesCollisionModel::createVoxelsModel(
    int abidx,
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Isometry3dVector& transforms)
{
    ROS_DEBUG_NAMED(ABM_LOGGER, "  Generate voxels model");

    // create configuration voxels model for this body
    CollisionVoxelModelConfig config;
    generateVoxelsModel(id, shapes, transforms, config);

    // append voxels models
    m_voxels_models.emplace_back(new CollisionVoxelsModel);
    ROS_DEBUG_NAMED(ABM_LOGGER, "  Voxels Model Count %zu -> %zu", m_voxels_models.size() - 1, m_voxels_models.size());

    CollisionVoxelsModel* voxels_model = m_voxels_models.back().get();

    // attach to the attached body
    voxels_model->link_index = abidx;

    const double AB_VOXEL_RES = 0.01;
    voxels_model->voxel_res = AB_VOXEL_RES;

    if (!voxelizeAttachedBody(shapes, transforms, *voxels_model)) {
        ROS_ERROR_NAMED(ABM_LOGGER, "Failed to voxelize attached body '%s'", id.c_str());
        // TODO: anything to do in this case
    }

    return voxels_model;
}

void AttachedBodiesCollisionModel::generateSpheresModel(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Isometry3dVector& transforms,
    CollisionSpheresModelConfig& spheres_model)
{
    assert(std::all_of(shapes.begin(), shapes.end(),
            [](const shapes::ShapeConstPtr& shape) { return (bool)shape; }));
    assert(shapes.size() == transforms.size());

    ROS_DEBUG_NAMED(ABM_LOGGER, "Generate spheres model configuration");

    // TODO: reserve a special character so as to guarantee sphere uniqueness
    // here and disallow use of the special character on config-generated
    // spheres

    // TODO: yeah...
    const double object_enclosing_sphere_radius = 0.025;

    // voxelize the object
    std::vector<Eigen::Vector3d> voxels;
    for (size_t i = 0; i < shapes.size(); ++i) {
        if (!VoxelizeShape(
                *shapes[i], transforms[i],
                object_enclosing_sphere_radius / std::sqrt(2),
                Eigen::Vector3d::Zero(),
                voxels))
        {
            ROS_ERROR_NAMED(ABM_LOGGER, "Failed to voxelize attached body shape for sphere generation");
            return;
        }
    }

    spheres_model.autogenerate = false;
    spheres_model.link_name = id;
    spheres_model.spheres.clear();

    for (size_t i = 0; i < voxels.size(); ++i) {
        CollisionSphereConfig sphere_config;
        sphere_config.name = id + "/" + std::to_string(i);
        sphere_config.x = voxels[i].x();
        sphere_config.y = voxels[i].y();
        sphere_config.z = voxels[i].z();
        sphere_config.radius = object_enclosing_sphere_radius;

        spheres_model.spheres.push_back(std::move(sphere_config));
    }

    ROS_DEBUG_NAMED(ABM_LOGGER, "Generated spheres model with %zu spheres", spheres_model.spheres.size());
}

void AttachedBodiesCollisionModel::generateVoxelsModel(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Isometry3dVector& transforms,
    CollisionVoxelModelConfig& voxels_model)
{
    ROS_DEBUG_NAMED(ABM_LOGGER, "Generate voxels model configuration");
    voxels_model.link_name = id;
}

bool AttachedBodiesCollisionModel::voxelizeAttachedBody(
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Isometry3dVector& transforms,
    CollisionVoxelsModel& model) const
{
    if (shapes.size() != transforms.size()) {
        ROS_ERROR_NAMED(ABM_LOGGER, "shapes array and transforms array must have equal length");
        return false;
    }

    std::vector<Eigen::Vector3d> voxels;
    for (size_t i = 0; i < shapes.size(); ++i) {
        const shapes::Shape& shape = *shapes[i];
        const Eigen::Affine3d& transform = transforms[i];
        VoxelizeShape(
                shape, transform,
                model.voxel_res, Eigen::Vector3d::Zero(), model.voxels);
    }

    return true;
}

inline
int AttachedBodiesCollisionModel::generateAttachedBodyIndex()
{
    // TODO: please don't add more than INT_MAX attached bodies over the
    // lifetime of this object
    if (m_attached_bodies_added ==
            std::numeric_limits<decltype(m_attached_bodies_added)>::max())
    {
        ROS_WARN_NAMED(ABM_LOGGER, "Oh god, it's happening");
    }
    return m_attached_bodies_added++;
}

} // namespace collision
} // namespace smpl
