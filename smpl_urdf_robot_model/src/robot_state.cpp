#include <smpl_urdf_robot_model/robot_state.h>

// standard includes
#include <assert.h>
#include <math.h>
#include <algorithm>
#include <limits>
#include <utility>

// project includes
#include <smpl_urdf_robot_model/robot_model.h>

namespace sbpl {
namespace motion {
namespace urdf {

static
Affine3 ComputeJointTransform(const Joint* joint, const double* variables)
{
    switch (joint->type) {
    case JointType::Fixed:
        return Affine3::Identity();
    case JointType::Revolute:
        return Affine3(AngleAxis(variables[0], joint->axis));
    case JointType::Prismatic:
        return Affine3(Translation3(variables[0] * joint->axis));
    case JointType::Planar:
        return Translation3(variables[0], variables[1], 0.0) *
                AngleAxis(variables[2], Vector3::UnitZ());
    case JointType::Floating:
        return Translation3(variables[0], variables[1], variables[2]) *
                Quaternion(variables[6], variables[3], variables[4], variables[5]);
    default:
        assert(0);
    }
}

static
void GetTransformVariables(
    const Joint* joint,
    const Affine3* transform,
    double* variables)
{
    switch (joint->type) {
    case JointType::Fixed:
        return;
    case JointType::Revolute:
    {
        Quaternion q(transform->rotation());
        size_t idx;
        joint->axis.array().abs().maxCoeff(&idx);
        variables[0] = 2.0 * atan2(q.vec()[idx] / joint->axis[idx], q.w());
        return;
    }
    case JointType::Prismatic:
    {
        auto val = transform->translation().dot(joint->axis);
        variables[0] = val;
        return;
    }
    case JointType::Planar:
    {
        variables[0] = transform->translation().x();
        variables[1] = transform->translation().y();

        Quaternion q(transform->rotation());
        auto s_squared = 1.0 - (q.w() * q.w());
        if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
            variables[2] = 0.0;
        } else {
            auto s = 1.0 / sqrt(s_squared);
            variables[2] = (2.0 * acos(q.w())) * (q.z() * s);
        }
        return;
    }
    case JointType::Floating:
    {
        variables[0] = transform->translation().x();
        variables[1] = transform->translation().y();
        variables[2] = transform->translation().z();
        Quaternion q(transform->rotation());
        variables[3] = q.x();
        variables[4] = q.y();
        variables[5] = q.z();
        variables[6] = q.w();
        return;
    }
    default:
        assert(0);
        return;
    }
}

bool Init(
    RobotState* out,
    const RobotModel* model,
    bool with_velocities,
    bool with_accelerations)
{
    RobotState robot_state;

    robot_state.model = model;

    int mult = 1;
    if (with_velocities) {
        ++mult;
    }
    if (with_accelerations) {
        ++mult;
    }
    robot_state.values.resize(mult * GetVariableCount(model));
    for (int i = 0; i < GetVariableCount(model); ++i) {
        robot_state.values[i] = GetDefaultPosition(model, GetVariable(model, i));
    }

    double* data = robot_state.values.data();
    robot_state.positions = data;
    data += GetVariableCount(model);
    if (with_velocities) {
        robot_state.velocities = data;
        data += GetVariableCount(model);
    }
    if (with_accelerations) {
        robot_state.accelerations = data;
        data += GetVariableCount(model);
    }

    robot_state.transforms.resize(
            GetLinkCount(model) +
            GetJointCount(model) +
            GetCollisionBodyCount(model) +
            GetVisualBodyCount(model));

    robot_state.link_transforms = robot_state.transforms.data();
    robot_state.joint_transforms = robot_state.link_transforms + GetLinkCount(model);
    robot_state.link_collision_transforms = robot_state.joint_transforms + GetJointCount(model);
    robot_state.link_visual_transforms = robot_state.link_collision_transforms + GetCollisionBodyCount(model);

    robot_state.dirty_links_joint = model->root_joint;
    robot_state.dirty_collisions_joint = model->root_joint;
    robot_state.dirty_visuals_joint = model->root_joint;

    *out = std::move(robot_state);
    return true;
}

auto GetRobotModel(const RobotState* state) -> const RobotModel*
{
    return state->model;
}

void SetToDefaultValues(RobotState* state)
{
    for (auto& v : Variables(state->model)) {
        auto position = GetDefaultPosition(state->model, &v);
        SetVariablePosition(state, &v, position);
    }
}

// Avoid setting all per-joint descendant bodies as dirty here => flag the
// most descendant dirty joint.
// Remove dependency on link transform for collision bodies and visual bodies.
// 3 dirty joints...for collision bodies, visual bodies, and links.
void SetVariablePositions(RobotState* state, const double* positions)
{
    for (int i = 0; i < GetVariableCount(state->model); ++i) {
        SetVariablePosition(state, i, positions[i]);
    }
}

void SetVariablePosition(
    RobotState* state,
    const JointVariable* variable,
    double position)
{
    return SetVariablePosition(
            state, GetVariableIndex(state->model, variable), position);
}

// TODO: maybe an overload of this that takes index and variable when both
// are already looked up
void SetVariablePosition(
    RobotState* state,
    int index,
    double position)
{
    if (state->positions[index] != position) {
        state->positions[index] = position;

        auto* v = GetVariable(state->model, index);
        auto* vj = GetJointOfVariable(v);

        if (state->dirty_links_joint == NULL) {
            state->dirty_links_joint = vj;
        } else {
            state->dirty_links_joint = GetCommonRoot(state->model, state->dirty_links_joint, vj);
        }
        if (state->dirty_collisions_joint == NULL) {
            state->dirty_collisions_joint = vj;
        } else {
            state->dirty_collisions_joint = GetCommonRoot(state->model, state->dirty_collisions_joint, vj);
        }
        if (state->dirty_visuals_joint == NULL) {
            state->dirty_visuals_joint = vj;
        } else {
            state->dirty_visuals_joint = GetCommonRoot(state->model, state->dirty_visuals_joint, vj);
        }
    }
}

auto GetVariablePositions(const RobotState* state) -> const double*
{
    return state->positions;
}

auto GetVariablePosition(const RobotState* state, const JointVariable* variable)
    -> double
{
    return GetVariablePosition(state, GetVariableIndex(state->model, variable));
}

auto GetVariablePosition(const RobotState* state, int index) -> double
{
    return state->positions[index];
}

bool HasVariableVelocities(const RobotState* state)
{
    return state->velocities != NULL;
}

void SetVariableVelocities(RobotState* state, const double* velocities)
{
    std::copy(velocities, velocities + GetVariableCount(state->model),
            state->velocities);
}

void SetVariableVelocity(
    RobotState* state,
    const JointVariable* variable,
    double v)
{
    return SetVariableVelocity(
            state, GetVariableIndex(state->model, variable), v);
}

void SetVariableVelocity(RobotState* state, int index, double v)
{
    state->velocities[index] = v;
}

auto GetVariableVelocities(const RobotState* state) -> const double*
{
    return state->velocities;
}

auto GetVariableVelocity(const RobotState* state, const JointVariable* variable)
    -> double
{
    return GetVariableVelocity(state, GetVariableIndex(state->model, variable));
}

auto GetVariableVelocity(const RobotState* state, int index) -> double
{
    return state->velocities[index];
}

bool HasVariableAccelerations(const RobotState* state)
{
    return state->accelerations != NULL;
}

void SetVariableAccelerations(RobotState* state, const double* accelerations)
{
    std::copy(accelerations, accelerations + GetVariableCount(state->model),
            state->accelerations);
}

void SetVariableAcceleration(
    RobotState* state,
    const JointVariable* variable,
    double a)
{
    return SetVariableAcceleration(
            state, GetVariableIndex(state->model, variable), a);
}

void SetVariableAcceleration(RobotState* state, int index, double a)
{
    state->accelerations[index] = a;
}

auto GetVariableAccelerations(const RobotState* state) -> const double*
{
    return state->accelerations;
}

auto GetVariableAcceleration(
    const RobotState* state,
    const JointVariable* variable) -> double
{
    return GetVariableAcceleration(
            state, GetVariableIndex(state->model, variable));
}

auto GetVariableAcceleration(const RobotState* state, int index) -> double
{
    return state->accelerations[index];
}

void SetJointPositions(
    RobotState* state,
    const Joint* joint,
    const double* positions)
{
    for (int i = 0; i < GetVariableCount(joint); ++i) {
        SetVariablePosition(state, GetFirstVariable(joint) + i, positions[i]);
    }
}

void SetJointPositions(RobotState* state, int index, const double* positions)
{
    return SetJointPositions(state, GetJoint(state->model, index), positions);
}

void SetJointPositions(
    RobotState* state,
    const Joint* joint,
    const Affine3* transform)
{
    double positions[7];
    GetTransformVariables(joint, transform, positions);
    return SetJointPositions(state, joint, positions);
}

void SetJointPositions(RobotState* state, int index, const Affine3* transform)
{
    return SetJointPositions(state, GetJoint(state->model, index), transform);
}

void SetJointVelocities(
    RobotState* state,
    const Joint* joint,
    const double* velocities)
{
    std::copy(
            velocities,
            velocities + GetVariableCount(joint),
            state->velocities + GetVariableIndex(state->model, GetFirstVariable(joint)));
}

void SetJointVelocities(RobotState* state, int index, const double* velocities)
{
    return SetJointVelocities(state, GetJoint(state->model, index), velocities);
}

void SetJointAccelerations(
    RobotState* state,
    const Joint* joint,
    const double* accelerations)
{
    std::copy(
            accelerations,
            accelerations + GetVariableCount(joint),
            state->accelerations + GetVariableIndex(state->model, GetFirstVariable(joint)));
}

void SetJointAccelerations(
    RobotState* state,
    int index,
    const double* accelerations)
{
    return SetJointAccelerations(state, GetJoint(state->model, index), accelerations);
}

static
auto GetJointPositions(RobotState* state, const Joint* joint) -> double*
{
    return state->positions + GetVariableIndex(state->model, GetFirstVariable(joint));
}

auto GetJointPositions(const RobotState* state, const Joint* joint)
    -> const double*
{
    return GetJointPositions(const_cast<RobotState*>(state), joint);
}

auto GetJointPositions(const RobotState* state, int index) -> const double*
{
    return GetJointPositions(state, GetJoint(state->model, index));
}

auto GetJointVelocities(const RobotState* state, const Joint* joint)
    -> const double*
{
    return state->velocities +
            GetVariableIndex(state->model, GetFirstVariable(joint));
}

auto GetJointVelocities(const RobotState* state, int index) -> const double*
{
    return GetJointVelocities(state, GetJoint(state->model, index));
}

auto GetJointAccelerations(const RobotState* state, const Joint* joint)
    -> const double*
{
    return state->accelerations +
            GetVariableIndex(state->model, GetFirstVariable(joint));
}

auto GetJointAccelerations(const RobotState* state, int index) -> const double*
{
    return GetJointAccelerations(state, GetJoint(state->model, index));
}

/////////////////////////// DANGER ZONE ///////////////////////////

static
void UpdateLinkTransforms(RobotState* state, std::vector<const Joint*>* q)
{
    assert(state->dirty_links_joint != NULL);

    q->push_back(state->dirty_links_joint);

    while (!q->empty()) {
        auto* joint = q->back();
        q->pop_back();

        auto* child_link = joint->child;

        // update the joint transform
        auto& joint_transform =
                state->joint_transforms[GetJointIndex(state->model, joint)];
        joint_transform =
                ComputeJointTransform(joint, GetJointPositions(state, joint));

        // update the child link transform
        auto& link_transform =
                state->link_transforms[GetLinkIndex(state->model, child_link)];
        if (joint->parent != NULL) {
            // parent_link * origin * joint transform
            auto& parent_transform =
                    state->link_transforms[
                            GetLinkIndex(state->model, joint->parent)];
            link_transform = parent_transform * joint->origin * joint_transform;
        } else {
            link_transform = joint->origin * joint_transform;
        }

        // recurse on children
        for (auto* child = child_link->children; child != NULL; child = child->sibling) {
            q->push_back(child);
        }
    }

    state->dirty_links_joint = NULL;
}

static
void UpdateOnlyCollisionBodyTransforms(
    RobotState* state,
    std::vector<const Joint*>* q)
{
    assert(state->dirty_collisions_joint != NULL);
    q->push_back(state->dirty_collisions_joint);
    while (!q->empty()) {
        auto* joint = q->back();
        q->pop_back();

        auto* child_link = joint->child;

        auto& link_transform =
                state->link_transforms[GetLinkIndex(state->model, child_link)];
        for (auto& collision : CollisionBodies(child_link)) {
            auto& collision_transform =
                    state->link_collision_transforms[
                            GetCollisionBodyIndex(state->model, &collision)];
            collision_transform = link_transform * collision.origin;
        }
        for (auto* child = child_link->children; child != NULL; child = child->sibling) {
            q->push_back(child);
        }
    }
    state->dirty_collisions_joint = NULL;
}

static
void UpdateOnlyVisualBodyTransforms(
    RobotState* state,
    std::vector<const Joint*>* q)
{
    assert(state->dirty_visuals_joint != NULL);
    q->push_back(state->dirty_visuals_joint);
    while (!q->empty()) {
        auto* joint = q->back();
        q->pop_back();

        auto* child_link = joint->child;

        auto& link_transform =
                state->link_transforms[GetLinkIndex(state->model, child_link)];
        for (auto& visual : VisualBodies(child_link)) {
            auto& visual_transform =
                    state->link_visual_transforms[
                            GetVisualBodyIndex(state->model, &visual)];
            visual_transform = link_transform * visual.origin;
        }
        for (auto* child = child_link->children; child != NULL; child = child->sibling) {
            q->push_back(child);
        }
    }
    state->dirty_visuals_joint = NULL;
}

void UpdateTransforms(RobotState* state)
{
    std::vector<const Joint*> q;
    if (state->dirty_links_joint != NULL) {
        UpdateLinkTransforms(state, &q);
    }
    if (state->dirty_collisions_joint != NULL) {
        UpdateOnlyCollisionBodyTransforms(state, &q);
    }
    if (state->dirty_visuals_joint != NULL) {
        UpdateOnlyVisualBodyTransforms(state, &q);
    }
}

void UpdateLinkTransforms(RobotState* state)
{
    if (state->dirty_links_joint != NULL) {
        std::vector<const Joint*> q;
        UpdateLinkTransforms(state, &q);
    }
}

void UpdateLinkTransform(RobotState* state, const Link* link)
{
    if (IsLinkTransformDirty(state, link)) {
        std::vector<const Joint*> q;
        UpdateLinkTransforms(state, &q);
    }
}

void UpdateLinkTransform(RobotState* state, int index)
{
    return UpdateLinkTransform(state, GetLink(state->model, index));
}

void UpdateCollisionBodyTransforms(RobotState* state)
{
    if (state->dirty_collisions_joint != NULL) {
        std::vector<const Joint*> q;
        if (state->dirty_links_joint != NULL) {
            // supertree or subtree => update
            if (IsAncestor(state->model, state->dirty_collisions_joint, state->dirty_links_joint) |
                IsAncestor(state->model, state->dirty_links_joint, state->dirty_collisions_joint))
            {
                UpdateLinkTransforms(state, &q);
            }
            // ...otherwise, sibling tree
        }
        UpdateOnlyCollisionBodyTransforms(state, &q);
    }
}

void UpdateCollisionBodyTransform(
    RobotState* state,
    const LinkCollision* collision)
{
    if (IsCollisionBodyTransformDirty(state, collision)) {
        std::vector<const Joint*> q;
        if (IsLinkTransformDirty(state, collision->link)) {
            UpdateLinkTransforms(state, &q);
        }
        UpdateOnlyCollisionBodyTransforms(state, &q);
    }
}

void UpdateCollisionBodyTransform(RobotState* state, int index)
{
    return UpdateCollisionBodyTransform(state, GetCollisionBody(state->model, index));
}

void UpdateVisualBodyTransforms(RobotState* state)
{
    if (state->dirty_visuals_joint != NULL) {
        std::vector<const Joint*> q;
        if (state->dirty_links_joint != NULL) {
            // supertree or subtree => update
            if (IsAncestor(state->model, state->dirty_visuals_joint, state->dirty_links_joint) |
                IsAncestor(state->model, state->dirty_links_joint, state->dirty_visuals_joint))
            {
                UpdateLinkTransforms(state, &q);
            }
            // ...otherwise, sibling tree
        }
        UpdateOnlyVisualBodyTransforms(state, &q);
    }
}

void UpdateVisualBodyTransform(RobotState* state, const LinkVisual* visual)
{
    if (IsVisualBodyTransformDirty(state, visual)) {
        std::vector<const Joint*> q;
        if (IsLinkTransformDirty(state, visual->link)) {
            UpdateLinkTransforms(state, &q);
        }
        UpdateOnlyVisualBodyTransforms(state, &q);
    }
}

void UpdateVisualBodyTransform(RobotState* state, int index)
{
    return UpdateVisualBodyTransform(state, GetVisualBody(state->model, index));
}

auto GetLinkTransform(const RobotState* state, const Link* link)
    -> const Affine3*
{
    return GetLinkTransform(state, GetLinkIndex(state->model, link));
}

auto GetLinkTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->link_transforms[index];
}

auto GetCollisionBodyTransform(
    const RobotState* state,
    const LinkCollision* collision)
    -> const Affine3*
{
    return GetCollisionBodyTransform(
            state, GetCollisionBodyIndex(state->model, collision));
}

auto GetCollisionBodyTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->link_collision_transforms[index];
}

auto GetVisualBodyTransform(
    const RobotState* state,
    const LinkVisual* visual)
    -> const Affine3*
{
    return GetVisualBodyTransform(
            state, GetVisualBodyIndex(state->model, visual));
}

auto GetVisualBodyTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->link_visual_transforms[index];
}

auto GetJointTransform(const RobotState* state, const Joint* joint)
    -> const Affine3*
{
    return GetJointTransform(state, GetJointIndex(state->model, joint));
}

auto GetJointTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->joint_transforms[index];
}

bool IsJointTransformDirty(const RobotState* state, const Joint* joint)
{
    return true;
}

bool IsLinkTransformDirty(const RobotState* state, const Link* link)
{
    return state->dirty_links_joint != NULL &&
            IsAncestor(state->model, state->dirty_links_joint, link->parent);
}

bool IsCollisionBodyTransformDirty(const RobotState* state, const LinkCollision* collision)
{
    // TODO: review
    return state->dirty_collisions_joint != NULL &&
            IsAncestor(state->model, state->dirty_collisions_joint, collision->link->parent);
}

bool IsVisualBodyTransformDirty(const RobotState* state, const LinkVisual* visual)
{
    return state->dirty_visuals_joint != NULL &&
            IsAncestor(state->model, state->dirty_visuals_joint, visual->link->parent);
}

bool IsDirty(const RobotState* state)
{
    return state->dirty_links_joint != NULL |
            state->dirty_visuals_joint != NULL |
            state->dirty_collisions_joint != NULL;
}

} // namespace urdf
} // namespace motion
} // namespace smpl

