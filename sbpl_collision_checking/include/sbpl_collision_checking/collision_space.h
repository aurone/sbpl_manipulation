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

#ifndef SBPL_COLLISION_CHECKING_COLLISION_SPACE_H
#define SBPL_COLLISION_CHECKING_COLLISION_SPACE_H

// standard includes
#include <string>
#include <memory>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_collision_checking/allowed_collisions_interface.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/self_collision_model.h>
#include <sbpl_collision_checking/world_collision_model.h>
#include <sbpl_collision_checking/types.h>

namespace smpl {
namespace collision {

class CollisionSpace : public CollisionChecker
{
public:

    ~CollisionSpace();

    bool init(
        OccupancyGrid* grid,
        const std::string& urdf_string,
        const CollisionModelConfig& config,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    bool init(
        OccupancyGrid* grid,
        const ::urdf::ModelInterface& urdf,
        const CollisionModelConfig& config,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    bool init(
        OccupancyGrid* grid,
        const RobotCollisionModelConstPtr& rcm,
        const std::string& group_name,
        const std::vector<std::string>& planning_joints);

    auto getPlanningVariables() const -> const std::vector<std::string>& {
        return m_planning_variables;
    }

    /// \name Robot State
    ///@{
    bool setJointPosition(const std::string& name, double position);
    void setWorldToModelTransform(const Eigen::Affine3d& transform);
    ///@}

    void setPadding(double padding);

    /// \name Self Collisions
    ///@{
    auto allowedCollisionMatrix() const -> const AllowedCollisionMatrix&;
    void updateAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
    void setAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
    ///@}

    /// \name World Collision Model
    ///@{
    bool insertObject(const CollisionObject* object);
    bool removeObject(const CollisionObject* object);
    bool moveShapes(const CollisionObject* object);
    bool insertShapes(const CollisionObject* object);
    bool removeShapes(const CollisionObject* object);
    ///@}

    /// \name Attached Objects
    ///@{
    bool attachObject(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Isometry3dVector& transforms,
        const std::string& link_name);

    bool detachObject(const std::string& id);
    ///@}

    auto getReferenceFrame() const -> const std::string&
    { return m_grid->getReferenceFrame(); }

    auto getGroupName() const -> const std::string& { return m_group_name; }

    auto robotCollisionModel() const -> const RobotCollisionModelConstPtr&
    { return m_rcm; }

    auto robotMotionCollisionModel() const
        -> const RobotMotionCollisionModelConstPtr&
    { return m_rmcm; }

    auto grid() -> OccupancyGrid* { return m_grid; }
    auto grid() const -> const OccupancyGrid* { return m_grid; }

    auto worldCollisionModel() const -> WorldCollisionModelConstPtr
    { return m_wcm; }

    auto selfCollisionModel() const -> SelfCollisionModelConstPtr
    { return m_scm; }

    /// \name Visualization
    ///@{
    auto getWorldVisualization() const -> visualization_msgs::MarkerArray;
    auto getRobotVisualization() const -> visualization_msgs::MarkerArray;

    auto getCollisionWorldVisualization() const
        -> visualization_msgs::MarkerArray;
    auto getCollisionRobotVisualization()
        -> visualization_msgs::MarkerArray;
    auto getCollisionRobotVisualization(const std::vector<double>& vals)
        -> visualization_msgs::MarkerArray;
    auto getCollisionRobotVisualization(const double* jvals)
        -> visualization_msgs::MarkerArray;

    auto getCollisionDetailsVisualization() const
        -> visualization_msgs::MarkerArray;
    auto getCollisionDetailsVisualization(const std::vector<double>& vals)
        -> visualization_msgs::MarkerArray;
    auto getCollisionDetailsVisualization(const double* jvals)
        -> visualization_msgs::MarkerArray;

    auto getBoundingBoxVisualization() const -> visual::Marker;
    auto getDistanceFieldVisualization() const -> visual::Marker;
    auto getOccupiedVoxelsVisualization() const -> visual::Marker;
    ///@}

    bool checkCollision(
        const std::vector<double>& state,
        const AllowedCollisionsInterface& aci,
        bool verbose,
        bool visualize,
        double& dist);

    bool checkCollision(const std::vector<double>& state, double& dist);
    bool checkCollision(const double* state, double& dist);

    double collisionDistance(const std::vector<double>& state);
    double collisionDistance(const double* state);

    bool collisionDetails(
        const std::vector<double>& state,
        CollisionDetails& details);
    bool collisionDetails(const double* state, CollisionDetails& details);

    /// \name Required Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Functions from CollisionChecker
    ///@{
    bool isStateValid(
        const RobotState& state,
        bool verbose = false) override;

    bool isStateToStateValid(
        const RobotState& start,
        const RobotState& finish,
        bool verbose = false) override;

    bool interpolatePath(
        const RobotState& start,
        const RobotState& finish,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name Reimplemented Functions from CollisionChecker
    ///@{
    auto getCollisionModelVisualization(const RobotState& vals)
        -> std::vector<visual::Marker> override;
    ///@}

public:

    OccupancyGrid*                  m_grid;
    std::vector<std::string>        m_planning_variables;

    RobotCollisionModelConstPtr         m_rcm;
    AttachedBodiesCollisionModelPtr     m_abcm;

    RobotMotionCollisionModelConstPtr   m_rmcm;

    RobotCollisionStatePtr          m_rcs;
    AttachedBodiesCollisionStatePtr m_abcs;
    std::vector<double>             m_joint_vars;

    WorldCollisionModelPtr          m_wcm;
    SelfCollisionModelPtr           m_scm;

    // Collision Group
    std::string                     m_group_name;
    int                             m_gidx = -1;

    // Planning Joint Information
    std::vector<int>                m_planning_joint_to_collision_model_indices;

    size_t planningVariableCount() const {
        return m_planning_joint_to_collision_model_indices.size();
    }

    bool isContinuous(int vidx) const;
    bool hasLimit(int vidx) const;
    double minLimit(int vidx) const;
    double maxLimit(int vidx) const;

    void updateState(const std::vector<double>& vals);
    void updateState(const double* vals);
    void updateState(
        std::vector<double>& state,
        const std::vector<double>& vals);
    void updateState(std::vector<double>& state, const double* vals);
    void copyState();

    bool withinJointPositionLimits(const std::vector<double>& positions) const;
};

typedef std::shared_ptr<CollisionSpace> CollisionSpacePtr;
typedef std::shared_ptr<const CollisionSpace> CollisionSpaceConstPtr;

auto BuildCollisionSpace(
    OccupancyGrid* grid,
    const std::string& urdf_string,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
    -> std::unique_ptr<CollisionSpace>;

auto BuildCollisionSpace(
    OccupancyGrid* grid,
    const ::urdf::ModelInterface& urdf,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
    -> std::unique_ptr<CollisionSpace>;

auto BuildCollisionSpace(
    OccupancyGrid* grid,
    const RobotCollisionModelConstPtr& rcm,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
    -> std::unique_ptr<CollisionSpace>;

///////////////////////////
// Inline Implementation //
///////////////////////////

inline
bool CollisionSpace::isContinuous(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarIsContinuous(jidx);
}

inline
bool CollisionSpace::hasLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarHasPositionBounds(jidx);
}

inline
double CollisionSpace::minLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarMinPosition(jidx);
}

inline
double CollisionSpace::maxLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_rcm->jointVarMaxPosition(jidx);
}

} // namespace collision
} // namespace smpl

#endif
