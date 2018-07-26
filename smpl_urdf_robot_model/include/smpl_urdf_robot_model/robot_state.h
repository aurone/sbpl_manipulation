#ifndef SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_H
#define SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_H

// standard includes
#include <vector>

// project includes
#include <smpl_urdf_robot_model/types.h>

namespace sbpl {
namespace motion {
namespace urdf {

struct RobotModel;
struct Joint;
struct Link;
struct JointVariable;
struct LinkCollision;
struct LinkVisual;

struct RobotState;

bool Init(
    RobotState* state,
    const RobotModel* model,
    bool with_velocities = false,
    bool with_accelerations = false);

auto GetRobotModel(const RobotState* state) -> const RobotModel*;

///////////////
// Positions //
///////////////

void SetToDefaultValues(RobotState* state);

void SetVariablePositions(RobotState* state, const double* positions);
void SetVariablePosition(RobotState* state, const JointVariable* variable, double position);
void SetVariablePosition(RobotState* state, int index, double position);

auto GetVariablePositions(const RobotState* state) -> const double*;
auto GetVariablePosition(const RobotState* state, const JointVariable* variable) -> double;
auto GetVariablePosition(const RobotState* state, int index) -> double;

////////////////
// Velocities //
////////////////

bool HasVariableVelocities(const RobotState* state);

void SetVariableVelocities(RobotState* state, const double* velocities);
void SetVariableVelocity(RobotState* state, const JointVariable* variable, double v);
void SetVariableVelocity(RobotState* state, int index, double v);

auto GetVariableVelocities(const RobotState* state) -> const double*;
auto GetVariableVelocity(const RobotState* state, const JointVariable* variable) -> double;
auto GetVariableVelocity(const RobotState* state, int index) -> double;

///////////////////
// Accelerations //
///////////////////

bool HasVariableAccelerations(const RobotState* state);

void SetVariableAccelerations(RobotState* state, const double* accelerations);
void SetVariableAcceleration(RobotState* state, const JointVariable* variable, double a);
void SetVariableAcceleration(RobotState* state, int index, double a);

auto GetVariableAccelerations(const RobotState* state) -> const double*;
auto GetVariableAcceleration(const RobotState* state, const JointVariable* variable) -> double;
auto GetVariableAcceleration(const RobotState* state, int index) -> double;

////////////
// Joints //
////////////

void SetJointPositions(RobotState* state, const Joint* joint, const double* positions);
void SetJointPositions(RobotState* state, int index, const double* positions);

void SetJointPositions(RobotState* state, const Joint* joint, const Affine3* transform);
void SetJointPositions(RobotState* state, int index, const Affine3* transform);

void SetJointVelocities(RobotState* state, const Joint* joint, const double* velocities);
void SetJointVelocities(RobotState* state, int index, const double* velocities);

void SetJointAccelerations(RobotState* state, const Joint* joint, const double* accelerations);
void SetJointAccelerations(RobotState* state, int index, const double* accelerations);

auto GetJointPositions(const RobotState* state, const Joint* joint) -> const double*;
auto GetJointPositions(const RobotState* state, int index) -> const double*;

auto GetJointVelocities(const RobotState* state, const Joint* joint) -> const double*;
auto GetJointVelocities(const RobotState* state, int index) -> const double*;

auto GetJointAccelerations(const RobotState* state, const Joint* joint) -> const double*;
auto GetJointAccelerations(const RobotState* state, int index) -> const double*;

////////////////
// Transforms //
////////////////

// Update transforms for all joints, links, collision bodies, and visual bodies.
void UpdateTransforms(RobotState* state);

// Update transforms for all links.
void UpdateLinkTransforms(RobotState* state);

// Update the transform of a specific link.
void UpdateLinkTransform(RobotState* state, const Link* link);
void UpdateLinkTransform(RobotState* state, int index);

// Update the transforms for all collision bodies.
void UpdateCollisionBodyTransforms(RobotState* state);

// Update the transform of a specific collision body.
void UpdateCollisionBodyTransform(RobotState* state, const LinkCollision* collision);
void UpdateCollisionBodyTransform(RobotState* state, int index);

// Update the transforms for all visual bodies.
void UpdateVisualBodyTransforms(RobotState* state);

// Update the transform for a specific visual body.
void UpdateVisualBodyTransform(RobotState* state, const LinkVisual* visual);
void UpdateVisualBodyTransform(RobotState* state, int index);

// Retrieve transforms.
auto GetLinkTransform(const RobotState* state, const Link* link) -> const Affine3*;
auto GetLinkTransform(const RobotState* state, int index) -> const Affine3*;
auto GetCollisionBodyTransform(const RobotState* state, const LinkCollision* collision) -> const Affine3*;
auto GetCollisionBodyTransform(const RobotState* state, int index) -> const Affine3*;
auto GetVisualBodyTransform(const RobotState* state, const LinkVisual* visual) -> const Affine3*;
auto GetVisualBodyTransform(const RobotState* state, int index) -> const Affine3*;
auto GetJointTransform(const RobotState* state, const Joint* joint) -> const Affine3*;
auto GetJointTransform(const RobotState* state, int joint) -> const Affine3*;

// Query whether the transform for a given entity is outdated.
bool IsJointTransformDirty(const RobotState* state, const Joint* joint);
bool IsLinkTransformDirty(const RobotState* state, const Link* link);
bool IsCollisionBodyTransformDirty(const RobotState* state, const LinkCollision* collision);
bool IsVisualBodyTransformDirty(const RobotState* state, const LinkVisual* visual);
bool IsDirty(const RobotState* state);

struct RobotState
{
    const RobotModel*       model = NULL;

    std::vector<double>     values;
    double*                 positions = NULL;
    double*                 velocities = NULL;
    double*                 accelerations = NULL;

    std::vector<Affine3>    transforms;
    Affine3*                link_transforms = NULL;
    Affine3*                joint_transforms = NULL;
    Affine3*                link_collision_transforms = NULL;
    Affine3*                link_visual_transforms = NULL;

    const Joint*            dirty_links_joint = NULL;
    const Joint*            dirty_collisions_joint = NULL;
    const Joint*            dirty_visuals_joint = NULL;

    // self-references => non-copyable
    RobotState() = default;
    RobotState(const RobotState&) = delete;
    RobotState(RobotState&&) = default;
    RobotState& operator=(const RobotState&) = delete;
    RobotState& operator=(RobotState&&) = default;
};

} // namespace urdf
} // namespace motion
} // namespace sbpl

#endif

