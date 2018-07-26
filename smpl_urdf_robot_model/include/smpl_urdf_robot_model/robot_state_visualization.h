#ifndef SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_VISUALIZATION_H
#define SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_VISUALIZATION_H

// standard includes
#include <vector>

// system includes
#include <smpl/debug/marker.h>

namespace sbpl {
namespace motion {
namespace urdf {

struct RobotState;

auto MakeRobotVisualization(
    const RobotState* state,
    sbpl::visual::Color color = sbpl::visual::Color{ 0.0f, 0.0f, 0.0f, 1.0f },
    const std::string& frame = std::string(),
    const std::string& ns = std::string(),
    int32_t* id = NULL)
    -> std::vector<sbpl::visual::Marker>;

auto MakeCollisionVisualization(
    const RobotState* state,
    sbpl::visual::Color color = sbpl::visual::Color{ 0.0f, 0.0f, 0.0f, 1.0f },
    const std::string& frame = std::string(),
    const std::string& ns = std::string(),
    int32_t* id = NULL)
    -> std::vector<sbpl::visual::Marker>;

} // namespace urdf
} // namespace motion
} // namespace sbpl

#endif
