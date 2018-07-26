#include <smpl_urdf_robot_model/robot_state_bounds.h>

// project includes
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>

namespace sbpl {
namespace motion {
namespace urdf {

bool SatisfiesBounds(const RobotState* state)
{
    for (int i = 0; i < GetVariableCount(state->model); ++i) {
        auto* bounds = GetVariableLimits(GetVariable(state->model, i));
        if (bounds->has_position_limits) {
            if (state->positions[i] < bounds->min_position |
                state->positions[i] > bounds->max_position)
            {
                return false;
            }
        }
    }
    return true;
}

bool SatisfiesBounds(const RobotState* state, const Joint* joint)
{
    for (int i = 0; i < GetVariableCount(joint); ++i) {
        auto* var = GetFirstVariable(joint) + i;
        if (!SatisfiesBounds(state, var)) return false;
    }

    return true;
}

bool SatisfiesBounds(const RobotState* state, const JointVariable* variable)
{
    auto* bounds = GetVariableLimits(variable);
    if (!bounds->has_position_limits) return true;

    auto index = GetVariableIndex(state->model, variable);
    return state->positions[index] >= bounds->min_position &
            state->positions[index] <= bounds->max_position;
}

} // namespace urdf
} // namespace motion
} // namespace sbpl

