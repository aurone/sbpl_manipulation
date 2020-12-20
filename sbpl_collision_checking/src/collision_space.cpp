////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <sbpl_collision_checking/collision_space.h>

// standard includes
#include <assert.h>
#include <limits>
#include <utility>
#include <queue>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <leatherman/print.h>
#include <leatherman/viz.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/debug/marker_conversions.h>
#include <urdf/model.h>

// project includes
#include <sbpl_collision_checking/shapes.h>

namespace smpl {
namespace collision {

static const char* LOG = "cspace";

/// \class CollisionSpace
///
/// This class provides a unified interface to a complete collision model for
/// checking robot state and motion validity. The collision model detects both
/// collisions with the world and self collisions.
///
/// Storage for world objects is managed within the CollisionSpace. Collision
/// objects may be inserted using either existing objects or built from
/// CollisionObject or OctomapWithPose messages. The collision objects are linked
/// to an underlying WorldCollisionModel to construct their volumetric
/// representation. In addition to disallowing duplicates, as is enforced by the
/// internal WorldCollisionModel, objects are also uniquely identified by their
/// string id's, so that no two objects with the same id can exist simultaneously
/// within the CollisionSpace. This is done so that multiple insertions of the
/// same CollisionObject or OctomapWithPose message do not create multiple
/// objects within the WorldCollisionModel.

CollisionSpace::~CollisionSpace()
{
}

/// \brief Set a joint variable in the robot state
/// \param The joint variable name
/// \param The joint variable position
/// \return true if the joint variable exists; false otherwise
bool CollisionSpace::setJointPosition(
    const std::string& name,
    double position)
{
    if (m_rcm->hasJointVar(name)) {
        int jidx = m_rcm->jointVarIndex(name);
        m_joint_vars[jidx] = position;
        return true;
    } else {
        return false;
    }
}

/// \brief Set the transform from the reference frame to the robot model frame
/// \param transform The transform from the reference frame to the robot frame
void CollisionSpace::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    m_rcs->setWorldToModelTransform(transform);
    const int vfidx = m_rcm->jointVarIndexFirst(0);
    const int vlidx = m_rcm->jointVarIndexLast(0);
    std::copy(
            m_rcs->getJointVarPositions() + vfidx,
            m_rcs->getJointVarPositions() + vlidx,
            m_joint_vars.data() + vfidx);
    m_scm->setWorldToModelTransform(transform);
}

/// \brief Set the padding applied to the collision model
void CollisionSpace::setPadding(double padding)
{
    m_wcm->setPadding(padding);
    m_scm->setPadding(padding);
}

/// \brief Return the allowed collision matrix
/// \return The allowed collision matrix
const AllowedCollisionMatrix& CollisionSpace::allowedCollisionMatrix() const
{
    return m_scm->allowedCollisionMatrix();
}

/// \brief Update the allowed collisoin matrix
/// \param acm The allowed collision matrix
void CollisionSpace::updateAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    return m_scm->updateAllowedCollisionMatrix(acm);
}

/// \brief Set the allowed collision matrix
/// \param acm The allowed collision matrix
void CollisionSpace::setAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    m_scm->setAllowedCollisionMatrix(acm);
}

/// \brief Insert an object into the world
/// \param object The object
/// \return true if the object was inserted; false otherwise
bool CollisionSpace::insertObject(const CollisionObject* object)
{
    if (!m_wcm->insertObject(object)) {
        ROS_WARN_NAMED(LOG, "Reject insertion of object '%s'. Failed to add to world collision model.", object->id.c_str());
        return false;
    }

    return true;
}

/// \brief Remove an object from the world
/// \param object The object
/// \return true if the object was removed; false otherwise
bool CollisionSpace::removeObject(const CollisionObject* object)
{
    // don't need to check against object name here since it would be redundant

    if (!m_wcm->removeObject(object)) {
        ROS_WARN_NAMED(LOG, "Reject removal of object '%s'. Failed to remove from world collision model.", object->id.c_str());
        return false;
    }

    return true;
}

/// \brief Move an object in the world
/// \param object The object to be moved, with its new shapes
/// \return true if the object was moved; false otherwise
bool CollisionSpace::moveShapes(const CollisionObject* object)
{
    return m_wcm->moveShapes(object);
}

/// \brief Append shapes to an object
/// \param object The object to append shapes to, with its new shapes
/// \return true if the shapes were appended to the object; false otherwise
bool CollisionSpace::insertShapes(const CollisionObject* object)
{
    return m_wcm->insertShapes(object);
}

/// \brief Remove shapes from an object
/// \param object The object to remove shapes from, with the shapes to be removed
/// \return true if the shapes were removed; false otherwise
bool CollisionSpace::removeShapes(const CollisionObject* object)
{
    return m_wcm->removeShapes(object);
}

/// \brief Attach a collision object to the robot
/// \param id The name of the object
/// \param shapes The shapes composing the object
/// \param transforms The pose of each shape with respect to the attached link
/// \param link_name The link to attach the object to
/// \return true if the object was attached; false otherwise
bool CollisionSpace::attachObject(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Isometry3dVector& transforms,
    const std::string& link_name)
{
    return m_abcm->attachBody(id, shapes, transforms, link_name);
}

/// \brief Detach a collision object from the robot
/// \param id The name of the object
/// \return true if the object was detached; false otherwise
bool CollisionSpace::detachObject(const std::string& id)
{
    return m_abcm->detachBody(id);
}

/// \brief Return a visualization of the current world
///
/// The visualization is of the set of collision object geometries
auto CollisionSpace::getWorldVisualization() const
    -> visualization_msgs::MarkerArray
{
    return m_wcm->getWorldVisualization();
}

/// \brief Return a visualization of the current robot state
///
/// The visualization is of the visual robot geometry
auto CollisionSpace::getRobotVisualization() const
    -> visualization_msgs::MarkerArray
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

/// \brief Return a visualization of the current collision world
///
/// The visualization is of the occupied voxels in the grid
auto CollisionSpace::getCollisionWorldVisualization() const
    -> visualization_msgs::MarkerArray
{
    return m_wcm->getCollisionWorldVisualization();
}

/// \brief Return a visualization of the current collision robot state
///
/// The visualization is of the robot bounding geometry
auto CollisionSpace::getCollisionRobotVisualization()
    -> visualization_msgs::MarkerArray
{
    // update the spheres within the group
    for (int ssidx : m_rcs->groupSpheresStateIndices(m_gidx)) {
        m_rcs->updateSphereStates(ssidx);
    }
    auto markers = m_rcs->getVisualization(m_gidx);
    for (auto& m : markers.markers) {
        m.header.frame_id = m_grid->getReferenceFrame();
    }

    // Add the attached collision object spheres
    for (int ssidx : m_abcs->groupSpheresStateIndices(m_gidx)) {
        m_abcs->updateSphereStates(ssidx);
    }
    auto markers_attached = m_abcs->getVisualization(m_gidx);
    for (auto& m : markers_attached.markers) {
        m.header.frame_id = m_grid->getReferenceFrame();
        markers.markers.push_back(m);
    }

    return markers;
}

/// \brief Return a visualization of the collision robot at a given state
auto CollisionSpace::getCollisionRobotVisualization(
    const std::vector<double>& vals)
    -> visualization_msgs::MarkerArray
{
    updateState(vals);
    // update the spheres within the group
    for (int ssidx : m_rcs->groupSpheresStateIndices(m_gidx)) {
        m_rcs->updateSphereStates(ssidx);
    }
    auto markers = m_rcs->getVisualization(m_gidx);
    for (auto& m : markers.markers) {
        m.header.frame_id = m_grid->getReferenceFrame();
    }
    return markers;
}

auto CollisionSpace::getCollisionRobotVisualization(const double* vals)
    -> visualization_msgs::MarkerArray
{
    updateState(vals);
    // update the spheres within the group
    for (int ssidx : m_rcs->groupSpheresStateIndices(m_gidx)) {
        m_rcs->updateSphereStates(ssidx);
    }
    auto markers = m_rcs->getVisualization(m_gidx);
    for (auto& m : markers.markers) {
        m.header.frame_id = m_grid->getReferenceFrame();
    }
    return markers;
}

/// \brief Return a visualization of the collision details for the current state
auto CollisionSpace::getCollisionDetailsVisualization() const
    -> visualization_msgs::MarkerArray
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

/// \brief Return a visualization of the collision details for a given state
auto CollisionSpace::getCollisionDetailsVisualization(
    const std::vector<double>& vals)
    -> visualization_msgs::MarkerArray
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

auto CollisionSpace::getCollisionDetailsVisualization(
    const double* vals)
    -> visualization_msgs::MarkerArray
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

/// \brief Return a visualization of the bounding box
auto CollisionSpace::getBoundingBoxVisualization() const -> visual::Marker
{
    return m_grid->getBoundingBoxVisualization();
}

/// \brief Return a visualization of the distance field
auto CollisionSpace::getDistanceFieldVisualization() const -> visual::Marker
{
    return m_grid->getDistanceFieldVisualization();
}

/// \brief Return a visualization of the occupied voxels
auto CollisionSpace::getOccupiedVoxelsVisualization() const -> visual::Marker
{
    return m_grid->getOccupiedVoxelsVisualization();
}

bool CollisionSpace::checkCollision(
    const std::vector<double>& state,
    const AllowedCollisionsInterface& aci,
    bool verbose,
    bool visualize,
    double& dist)
{
    // see note in checkCollision(const std::vector<double>&, double&)
    updateState(state);
    dist = std::numeric_limits<double>::max();
    return m_scm->checkCollision(*m_rcs, *m_abcs, aci, m_gidx, dist);
}

bool CollisionSpace::checkCollision(
    const std::vector<double>& state,
    double& dist)
{
    // NOTE: world collisions are implicitly checked via the self collision
    // model since the two models share the same occupancy grid, which the
    // self collision model uses to check for collisions against voxels groups
    updateState(state);
    return m_scm->checkCollision(*m_rcs, *m_abcs, m_gidx, dist);
}

bool CollisionSpace::checkCollision(const double* state, double& dist)
{
    updateState(state);
    return m_scm->checkCollision(*m_rcs, *m_abcs, m_gidx, dist);
}

double CollisionSpace::collisionDistance(const std::vector<double>& state)
{
    updateState(state);
    return m_scm->collisionDistance(*m_rcs, *m_abcs, m_gidx);
}

double CollisionSpace::collisionDistance(const double* state)
{
    updateState(state);
    return m_scm->collisionDistance(*m_rcs, *m_abcs, m_gidx);
}

bool CollisionSpace::collisionDetails(
    const std::vector<double>& state,
    CollisionDetails& details)
{
    updateState(state);
    return m_scm->collisionDetails(*m_rcs, *m_abcs, m_gidx, details);
}

bool CollisionSpace::collisionDetails(
    const double* state,
    CollisionDetails& details)
{
    updateState(state);
    return m_scm->collisionDetails(*m_rcs, *m_abcs, m_gidx, details);
}

Extension* CollisionSpace::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<CollisionChecker>()) {
        return this;
    }
    return nullptr;
}

bool CollisionSpace::isStateValid(const RobotState& state, bool verbose)
{
    double dist = std::numeric_limits<double>::max();
    return checkCollision(state, dist);
}

bool CollisionSpace::isStateToStateValid(
    const RobotState& start,
    const RobotState& finish,
    bool verbose)
{
    const double res = 0.05;

    MotionInterpolation interp(m_rcm.get());

    m_rmcm->fillMotionInterpolation(
            start,
            finish,
            m_planning_joint_to_collision_model_indices,
            res,
            interp);

    const int inc_cc = 5;

    RobotState interm;

    // TODO: Looks like the idea here is to collision check the path starting at
    // the most coarse resolution (just the endpoints) and increasing the
    // granularity until all points are checked. This could probably be made
    // irrespective of the number of waypoints by bisecting the path and
    // checking the endpoints recursively.
    if (interp.waypointCount() > inc_cc) {
        // try to find collisions that might come later in the path earlier
        for (int i = 0; i < inc_cc; i++) {
            for (size_t j = i; j < interp.waypointCount(); j = j + inc_cc) {
                interp.interpolate(j, interm, m_planning_joint_to_collision_model_indices);
                if (!isStateValid(interm, verbose)) {
                    return false;
                }
            }
        }
    } else {
        for (size_t i = 0; i < interp.waypointCount(); i++) {
            interp.interpolate(i, interm, m_planning_joint_to_collision_model_indices);
            if (!isStateValid(interm, verbose)) {
                return false;
            }
        }
    }

    return true;
}

bool CollisionSpace::interpolatePath(
    const RobotState& start,
    const RobotState& finish,
    std::vector<RobotState>& opath)
{
    assert(start.size() == m_planning_joint_to_collision_model_indices.size() &&
            finish.size() == m_planning_joint_to_collision_model_indices.size());

    // check joint limits on the start and finish points
    if (!withinJointPositionLimits(start) ||
        !withinJointPositionLimits(finish))
    {
        ROS_ERROR_NAMED(LOG, "Joint limits violated");
        return false;
    }

    const double res = 0.05;

    MotionInterpolation interp(m_rcm.get());
    m_rmcm->fillMotionInterpolation(
            start,
            finish,
            m_planning_joint_to_collision_model_indices,
            res,
            interp);
    opath.resize(interp.waypointCount());
    for (int i = 0; i < interp.waypointCount(); ++i) {
        opath[i].resize(m_planning_joint_to_collision_model_indices.size());
        interp.interpolate(i, opath[i], m_planning_joint_to_collision_model_indices);
    }

    return true;
}

auto CollisionSpace::getCollisionModelVisualization(const RobotState& state)
    -> std::vector<visual::Marker>
{
    auto ma = getCollisionRobotVisualization(state);
    std::vector<visual::Marker> markers;
    markers.reserve(ma.markers.size());
    visual::Marker m;
    for (auto& mm : ma.markers) {
        visual::ConvertMarkerMsgToMarker(mm, m);
        markers.push_back(m);
    }
    return markers;
}

/// \brief Initialize the Collision Space
/// \param urdf_string String description of the robot in URDF format
/// \param config Collision model configuration
/// \param group_name The group for which collision detection is performed
/// \param planning_joints The set of joint variable names in the order they
///     will appear in calls to isStateValid and friends
bool CollisionSpace::init(
    OccupancyGrid* grid,
    const std::string& urdf_string,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    auto urdf = boost::make_shared<::urdf::Model>();
    if (!urdf->initString(urdf_string)) {
        ROS_ERROR_NAMED(LOG, "Failed to parse URDF");
        return false;
    }

    return init(grid, *urdf, config, group_name, planning_joints);
}

/// \brief Initialize the Collision Space
/// \param urdf The URDF of the robot
/// \param config Collision model configuration
/// \param group_name Group for which collision detection is performed
/// \param planning_joints The set of joint variable names in the order they
///     will appear in calls to isStateValid and friends
bool CollisionSpace::init(
    OccupancyGrid* grid,
    const ::urdf::ModelInterface& urdf,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    ROS_DEBUG_NAMED(LOG, "Initializing collision space for group '%s'", group_name.c_str());

    auto rcm = RobotCollisionModel::Load(urdf, config);
    return init(grid, rcm, group_name, planning_joints);
}

/// \brief Initialize the Collision Space
/// \param rcm The robot collision model
/// \param group_name The group for which collision detection is performed
/// \param planning_joints The set of joint variable names in the order they
///     will appear in calls to isStateValid and friends
bool CollisionSpace::init(
    OccupancyGrid* grid,
    const RobotCollisionModelConstPtr& rcm,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    m_grid = grid;
    m_rcm = rcm;
    if (!m_rcm) {
        ROS_ERROR_NAMED(LOG, "Failed to initialize the Robot Collision Model");
        return false;
    }

    for (auto& joint_name : planning_joints) {
        if (!m_rcm->hasJointVar(joint_name)) {
            ROS_ERROR_NAMED(LOG, "Joint variable '%s' not found in Robot Collision Model", joint_name.c_str());
            return false;
        }
    }

    // map planning joint indices to collision model indices
    m_planning_joint_to_collision_model_indices.resize(planning_joints.size(), -1);

    for (size_t i = 0; i < planning_joints.size(); ++i) {
        auto& joint_name = planning_joints[i];;
        int jidx = m_rcm->jointVarIndex(joint_name);

        m_planning_joint_to_collision_model_indices[i] = jidx;
    }

    if (!m_rcm->hasGroup(group_name)) {
        ROS_ERROR_NAMED(LOG, "Group '%s' was not found in the Robot Collision Model", group_name.c_str());
        return false;
    }

    m_planning_variables = planning_joints;
    m_group_name = group_name;
    m_gidx = m_rcm->groupIndex(m_group_name);

    m_rmcm = std::make_shared<RobotMotionCollisionModel>(m_rcm.get());
    m_abcm = std::make_shared<AttachedBodiesCollisionModel>(m_rcm.get()),
    m_rcs = std::make_shared<RobotCollisionState>(m_rcm.get());
    m_abcs = std::make_shared<AttachedBodiesCollisionState>(m_abcm.get(), m_rcs.get());
    m_wcm = std::make_shared<WorldCollisionModel>(m_grid);
    m_scm = std::make_shared<SelfCollisionModel>(m_grid, m_rcm.get(), m_abcm.get());

    m_joint_vars.assign(
        m_rcs->getJointVarPositions(),
        m_rcs->getJointVarPositions() + m_rcm->jointVarCount());

    return true;
}

void CollisionSpace::updateState(const std::vector<double>& vals)
{
    updateState(m_joint_vars, vals);
    copyState();
}

void CollisionSpace::updateState(const double* state)
{
    updateState(m_joint_vars, state);
    copyState();
}

void CollisionSpace::updateState(
    std::vector<double>& state,
    const std::vector<double>& vals)
{
    assert(vals.size() == m_planning_joint_to_collision_model_indices.size());
    updateState(state, vals.data());
}

void CollisionSpace::updateState(
    std::vector<double>& state,
    const double* vals)
{
    for (size_t i = 0; i < m_planning_joint_to_collision_model_indices.size(); ++i) {
        int jidx = m_planning_joint_to_collision_model_indices[i];
        state[jidx] = vals[i];
    }
}

void CollisionSpace::copyState()
{
    m_rcs->setJointVarPositions(m_joint_vars.data());
}

/// \brief Check whether the planning joint variables are within limits
/// \return true if all variables are within limits; false otherwise
bool CollisionSpace::withinJointPositionLimits(
    const std::vector<double>& positions) const
{
    assert(positions.size() == planningVariableCount());
    for (size_t vidx = 0; vidx < planningVariableCount(); ++vidx) {
        const double pos = positions[vidx];
        if (!(isContinuous(vidx) ||
            !hasLimit(vidx) ||
            (pos >= minLimit(vidx) && pos <= maxLimit(vidx))))
        {
            ROS_ERROR_NAMED(LOG, "Joint %zu at position %f outside limits [%f, %f]", vidx, pos, minLimit(vidx), maxLimit(vidx));
            return false;
        }
    }
    return true;
}

auto BuildCollisionSpace(
    OccupancyGrid* grid,
    const std::string& urdf_string,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
    -> std::unique_ptr<CollisionSpace>
{
    std::unique_ptr<CollisionSpace> cspace(new CollisionSpace);
    if (cspace->init(grid, urdf_string, config, group_name, planning_joints)) {
        return cspace;
    } else {
        return std::unique_ptr<CollisionSpace>();
    }
}

auto BuildCollisionSpace(
    OccupancyGrid* grid,
    const ::urdf::ModelInterface& urdf,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
    -> std::unique_ptr<CollisionSpace>
{
    std::unique_ptr<CollisionSpace> cspace(new CollisionSpace);
    if (cspace->init(grid, urdf, config, group_name, planning_joints)) {
        return cspace;
    } else {
        return std::unique_ptr<CollisionSpace>();
    }
}

auto BuildCollisionSpace(
    OccupancyGrid* grid,
    const RobotCollisionModelConstPtr& rcm,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
    -> std::unique_ptr<CollisionSpace>
{
    std::unique_ptr<CollisionSpace> cspace(new CollisionSpace);
    if (cspace->init(grid, rcm, group_name, planning_joints)) {
        return cspace;
    } else {
        return std::unique_ptr<CollisionSpace>();
    }
}

} // namespace collision
} // namespace smpl
