#ifndef SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_BOUNDS_H
#define SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_BOUNDS_H

namespace sbpl {
namespace motion {
namespace urdf {

struct RobotState;
struct Joint;
struct JointVariable;

bool SatisfiesBounds(const RobotState* state);
bool SatisfiesBounds(const RobotState* state, const Joint* joint);
bool SatisfiesBounds(const RobotState* state, const JointVariable* variable);

} // namespace urdf
} // namespace motion
} // namespace sbpl

#endif

