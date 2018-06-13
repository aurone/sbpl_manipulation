////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush, Fahad Islam
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
/// \author Fahad Islam

#include <smpl/graph/workspace_lattice_zero.h>

// system includes
#include <chrono>
#include <boost/functional/hash.hpp>
#include <fstream>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace sbpl {
namespace motion {

WorkspaceLatticeZero::~WorkspaceLatticeZero()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();

    // NOTE: StateID2IndexMapping cleared by DiscreteSpaceInformation
}

bool WorkspaceLatticeZero::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const PlanningParams* pp,
    const Params& _params)
{
    if (!WorkspaceLatticeBase::init(_robot, checker, pp, _params)) {
        return false;
    }

    // this should serve as a reasonable dummy state since no valid state should
    // have an empty coordinate vector

    // Doing it in FindRegionContainingState()
    //     WorkspaceCoord fake_coord;
    //     m_goal_state_id = createState(fake_coord);
    //     m_goal_entry = getState(m_goal_state_id);
    // }

    // SMPL_DEBUG_NAMED(pp->graph_log, "  goal state has id %d", m_goal_state_id);

    SMPL_DEBUG_NAMED(pp->graph_log, "initialize environment");

    if (!initMotionPrimitives()) {
        return false;
    }

    // Start Region
    m_min_ws_limits.resize(6 + freeAngleCount());
    m_max_ws_limits.resize(6 + freeAngleCount());

    double f = 0.0;//M_PI/24;

    m_min_ws_limits[0] =  0.5 - .13;
    m_min_ws_limits[1] =  -0.25;
    m_min_ws_limits[2] =  0.65;
    m_min_ws_limits[3] =  M_PI;
    m_min_ws_limits[4] =  0.0 - f;
    m_min_ws_limits[5] =  M_PI/2 - f;   //M_PI/2
    m_min_ws_limits[6] =  -2.135398; //-0.0872665;

    m_max_ws_limits[0] =  0.5 + .13;
    m_max_ws_limits[1] =  0.05;
    m_max_ws_limits[2] =  0.65;
    m_max_ws_limits[3] =  M_PI;     // normalization issue
    m_max_ws_limits[4] =  0.0 + f;
    m_max_ws_limits[5] =  M_PI/2 + f; // + 0.174533;
    m_max_ws_limits[6] =  0.564602; //-0.0872665;

    // normalize [-pi,pi]x[-pi/2,pi/2]x[-pi,pi]
    normalizeEulerAngles(&m_min_ws_limits[3]);
    normalizeEulerAngles(&m_max_ws_limits[3]);

    // center of cell
    WorkspaceCoord limits_coord(6 + freeAngleCount());
    stateWorkspaceToCoord(m_min_ws_limits, limits_coord);
    stateCoordToWorkspace(limits_coord, m_min_ws_limits);

    stateWorkspaceToCoord(m_max_ws_limits, limits_coord);
    stateCoordToWorkspace(limits_coord, m_max_ws_limits);

    m_distribution.resize(6 + freeAngleCount());

    for (int i = 0; i < m_distribution.size() ; ++i) {
        m_distribution[i] = std::uniform_real_distribution<double> (m_min_ws_limits[i], m_max_ws_limits[i]);
    }

    m_ik_amp_enabled = false;

    return true;
}

bool WorkspaceLatticeZero::projectToPose(int state_id, Eigen::Affine3d& pose)
{
    // if (state_id == getGoalStateID()) {
    //     pose = goal().tgt_off_pose;
    //     return true;
    // }

    WorkspaceLatticeState* state = getState(state_id);

    double p[6];
    poseCoordToWorkspace(&state->coord[0], &p[0]);

    pose = Eigen::Translation3d(p[0], p[1], p[2]) *
            Eigen::AngleAxisd(p[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(p[3], Eigen::Vector3d::UnitX());
    return true;
}

const WorkspaceState& WorkspaceLatticeZero::extractState(int state_id)
{
    stateCoordToWorkspace(m_states[state_id]->coord, m_ws_state);
    return m_ws_state;
}

void WorkspaceLatticeZero::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{
    GetSuccs(state_id, preds, costs);
    // SMPL_WARN("GetPreds unimplemented");
}

bool WorkspaceLatticeZero::IsStateValid(int state_id)
{
    // return true;
    WorkspaceLatticeState* entry = getState(state_id);
    // IK
    WorkspaceState ws_state;
    stateCoordToWorkspace(entry->coord, ws_state);
    if (!stateWorkspaceToRobot(ws_state, m_ik_seed, entry->state)) {
        return false;
    }

    if (!collisionChecker()->isStateValid(
        entry->state, true)) {
        // ROS_WARN("%d : In Collision", state_id);
        return false;
    }

    // no need right?
    // if (!robot()->checkJointLimits(entry->state)) {
    //     return false;
    // }
    return true;
}

bool WorkspaceLatticeZero::IsStateToStateValid(int from_state_id, int to_state_id)
{
    if (!IsStateValid(from_state_id)) {
        return false;
    }

    WorkspaceLatticeState* from_entry = getState(from_state_id);
    WorkspaceLatticeState* to_entry = getState(to_state_id);

    assert(from_entry->state == robot()->jointVariableCount());

// #if 0   // tie breaking
    if (to_entry->state.empty()) {
        WorkspaceState ws_state;
        stateCoordToWorkspace(to_entry->coord, ws_state);
        RobotState irstate;
        if (!stateWorkspaceToRobot(ws_state, m_ik_seed, to_entry->state)) {
            return false;
        }
    }
// #endif

    if (!collisionChecker()->isStateToStateValid(from_entry->state, to_entry->state)) {
        return false;
    }
    return true;
}

void WorkspaceLatticeZero::PruneRegions(std::vector<region>& regions)
{
    // prune
    std::vector<region> filtered_regions;
    for (const auto& r1 : regions) {
        bool keep = true;
        for (const auto& r2 : regions) {
            double eps = 0.0001;
            auto h = heuristic(0);
            WorkspaceCoord c1, c2;
            stateRobotToCoord(r1.state, c1);
            stateRobotToCoord(r2.state, c2);
            int id1 = createState(c1);
            int id2 = createState(c2);
            int d = h->GetFromToHeuristic(id1, id2);
            if (h->GetFromToHeuristic(id1, id2) == 0) {
                continue;
            }
            if (r2.radius > d + r1.radius) {
                keep = false;
                break;
            }
        }
        if (keep) {
            filtered_regions.push_back(r1);
        }
    }
    ROS_INFO("Original regions: %zu, filtered regions: %zu",
        regions.size(), filtered_regions.size());
    regions = filtered_regions;
}

int WorkspaceLatticeZero::SampleAttractorState(
    const std::vector<region>& regions,
    RobotState& robot_state,
    int max_tries)
{
    int attractor_state_id;
    int count = 0;
    while (count < max_tries) {
        count++;

        if (!SampleRobotState(robot_state)) {
            continue;
        }
        WorkspaceState ws_state;
        WorkspaceCoord ws_coord;
        stateRobotToCoord(robot_state, ws_coord);
        stateCoordToWorkspace(ws_coord, ws_state);
        int attractor_state_id = createState(ws_coord);

        // Check if in an existing region
        bool covered = false;
        for (const auto& r : regions) {
            WorkspaceState c_ws_state;
            WorkspaceCoord coord;
            stateRobotToCoord(r.state, coord);
            stateCoordToWorkspace(coord, c_ws_state);
            int center_state_id = createState(coord);
            RobotHeuristic* h = heuristic(0);   //TODO: Pick the correct heuristic the right way
            int dsum = h->GetFromToHeuristic(attractor_state_id, center_state_id);
            if (dsum < r.radius || dsum == 0) {
                covered = true;
                break;
            }
        }
        if (covered) {
            SMPL_DEBUG_NAMED(params()->graph_log, "State is covered already on try %d", count);
            continue;
        }

        SMPL_DEBUG_NAMED(params()->graph_log, "Sampled Attractor State");
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "    workspace state: " << ws_state);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "    workspace coord: " << ws_coord);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "    joint state    : " << robot_state);

        WorkspaceLatticeState* entry = getState(attractor_state_id);
        entry->state = robot_state;

        m_goal_state_id = attractor_state_id;  // for WorkspaceDistHeuristic

        // for heuristic
        m_near_goal = false;
        m_t_start = clock::now();
        // set the (modified) goal
        GoalConstraint gc = m_goal;     //may not be required but just in case
        gc.angles = ws_state;
        gc.type = GoalType::JOINT_STATE_GOAL;
        if (!RobotPlanningSpace::setGoal(gc)) {
            ROS_ERROR("Set new attractor goal failed");
        }

        auto* vis_name = "attractor_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(robot_state, vis_name));

        m_ik_seed = robot_state;

        ROS_INFO("Sampled attractor %zu on try: %d", regions.size(), count);

        return attractor_state_id;
    }

    return -1;
}

bool WorkspaceLatticeZero::SampleRobotState(RobotState& state)
{
    std::vector<double> ws_state(6 + freeAngleCount());
    for (int i = 0; i < ws_state.size() ; ++i) {
        ws_state[i] = m_distribution[i](m_generator);
    }

    // normalize and project to center
    normalizeEulerAngles(&ws_state[3]);
    WorkspaceCoord ws_coord;
    stateWorkspaceToCoord(ws_state, ws_coord);
    stateCoordToWorkspace(ws_coord, ws_state);

    if (!stateWorkspaceToRobot(ws_state, state)) {
        SMPL_DEBUG_NAMED(params()->graph_log, "Invalid IK on try");
        return false;
    }

    // Collision check`
    if (!collisionChecker()->isStateValid(state, true)) {
        SMPL_DEBUG_NAMED(params()->graph_log, "State in collision on try");
        return false;
    }

    SMPL_DEBUG_NAMED(params()->graph_log, "Sampled State");
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "    workspace state: " << ws_state);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "    workspace coord: " << ws_coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "    joint state    : " << state);

    return true;
}

int WorkspaceLatticeZero::FindRegionContainingState(
    const RobotState& state,
    const std::vector<region>& regions)
{
    WorkspaceState ws_state;
    WorkspaceCoord ws_coord;
    stateRobotToCoord(state, ws_coord);
    stateCoordToWorkspace(ws_coord, ws_state);
    int query_state_id = createState(ws_coord);

    // Collision check`
    if (!collisionChecker()->isStateValid(state, true)) {
        ROS_ERROR("Start state is in collision");
        return -1;
    }

    bool covered = false;
    int reg_idx = 0;
    WorkspaceState goal_state;
    for (const auto& r : regions) {
        WorkspaceState c_ws_state;
        WorkspaceCoord coord;
        stateRobotToCoord(r.state, coord);
        stateCoordToWorkspace(coord, c_ws_state);
        int center_state_id = createState(coord);
        RobotHeuristic* h = heuristic(0);
        int dsum = h->GetFromToHeuristic(query_state_id, center_state_id);
        // printf("dsum %d radius %u\n", dsum, r.radius);
        if (dsum < r.radius || dsum == 0) {
            // ROS_INFO("Covered try %d", count);
            goal_state = c_ws_state;
            covered = true;
            break;
        }
        reg_idx++;
    }

    if (covered) {
        SMPL_DEBUG_NAMED(params()->graph_log, "Attractor State of Containing Region");
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "    workspace state  : " << goal_state);
        return reg_idx;
    }
    else {
        SMPL_INFO_STREAM_NAMED(params()->expands_log, "  start ws_state: " << ws_state);
        return -1;
    }
}


bool WorkspaceLatticeZero::IsRobotStateInStartRegion(const RobotState& state)
{
    WorkspaceState ws_state;
    stateRobotToWorkspace(state, ws_state);
    return IsWorkspaceStateInStartRegion(ws_state);
}

bool WorkspaceLatticeZero::IsWorkspaceStateInStartRegion(const WorkspaceState& state)
{
    double eps = 0.0001;
    for (int i = 0; i < state.size(); ++i) {
        if (state[i] < m_min_ws_limits[i] - eps || state[i] > m_max_ws_limits[i] + eps) {
            SMPL_DEBUG_NAMED(params()->expands_log, "         -> violates start region limits %d", i);
            return false;
        }
    }
    return true;
}

bool WorkspaceLatticeZero::setGoal(const GoalConstraint& goal)
{
    m_goal = goal;

    bool res = false;
    if (goal.type == GoalType::XYZ_RPY_GOAL) {
        return setGoalPose(goal);
    } else {
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  goal: " << goal.angles);
        WorkspaceState goal_state;
        WorkspaceCoord goal_coord;
        stateRobotToCoord(goal.angles, goal_coord);
        stateCoordToWorkspace(goal_coord, goal_state);
        GoalConstraint gc;
        gc.angles = goal_state;
        gc.type = GoalType::JOINT_STATE_GOAL;
        if (!RobotPlanningSpace::setGoal(gc)) {
            ROS_ERROR("Set new goal goal failed");
            return false;
        }
        m_goal_state_id = createState(goal_coord);

        // set ik seed
        m_ik_seed = goal.angles;
    }

    return true;
}


bool WorkspaceLatticeZero::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    path.clear();

    if (ids.empty()) {
        return true;
    }

    if (ids.size() == 1) {
        const int state_id = ids[0];
        if (state_id == getGoalStateID()) {
            const WorkspaceLatticeState* entry = getState(m_start_state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", m_start_state_id);
                return false;
            }
            path.push_back(entry->state);
        } else {
            const WorkspaceLatticeState* entry = getState(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
        return true;
    }

    WorkspaceLatticeState* start_entry = getState(ids[0]);
    path.push_back(start_entry->state);

    for (size_t i = 1; i < ids.size(); ++i) {
        const int prev_id = ids[i - 1];
        const int curr_id = ids[i];

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(params()->graph_log, "cannot determine goal state successors during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            // TODO: variant of get succs that returns unique state ids
            WorkspaceLatticeState* prev_entry = getState(prev_id);
            std::vector<Action> actions;
            getActions(*prev_entry, actions);

            WorkspaceLatticeState* best_goal_entry = nullptr;
            int best_cost = std::numeric_limits<int>::max();

            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                const WorkspaceState& final_state = action.back();
                if (!isGoal(final_state)) {
                    continue;
                }

                if (!checkAction(prev_entry->state, action)) {
                    continue;
                }

                WorkspaceCoord goal_coord;
                stateWorkspaceToCoord(final_state, goal_coord);

                int goal_id = createState(goal_coord);
                WorkspaceLatticeState* goal_state = getState(goal_id);

                // shouldn't have created a new state, so no need to set the
                // continuous state counterpart
                assert(goal_state->state.size() == robot()->jointVariableCount());

                best_cost = 30; // Hardcoded primitive value in GetSuccs
                best_goal_entry = goal_state;
                break;
            }

            if (!best_goal_entry) {
                SMPL_ERROR_NAMED(params()->graph_log, "failed to find valid goal successor during path extraction");
                return false;
            }

            path.push_back(best_goal_entry->state);
        } else {
            WorkspaceLatticeState* state_entry = getState(curr_id);
            path.push_back(state_entry->state);
        }
    }

    return true;
}

Extension* WorkspaceLatticeZero::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<WorkspaceLatticeZero>() ||
        class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<PoseProjectionExtension>() ||
        class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>())
    {
        return this;
    }
    return nullptr;
}

void WorkspaceLatticeZero::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    // clear the successor arrays
    succs->clear();
    costs->clear();

    SMPL_DEBUG_NAMED(params()->expands_log, "Expand state %d", state_id);

    // goal state should be absorbing
    // if (state_id == m_goal_state_id) {
    //     return;
    // }

    WorkspaceLatticeState* parent_entry = getState(state_id);

    assert(parent_entry);
    assert(parent_entry->coord.size() == m_dof_count);

    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  state: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));
    // getchar();

#if 1
    int hue;
    if (IsStateValid(createState(parent_entry->coord))) {
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));
        hue = 30;
    }
    else {
        hue = 0;
    }
    WorkspaceState ws_parent;
    stateCoordToWorkspace(parent_entry->coord, ws_parent);
    // SMPL_INFO_STREAM_NAMED(params()->expands_log, "  state: " << ws_parent);
    vis_name = "workspace_expansion";
    auto marker = visual::MakeSphereMarker(ws_parent[0],
                                           ws_parent[1],
                                           ws_parent[2],
                                           m_res[0]/2,
                                           hue,
                                           m_viz_frame_id,
                                           vis_name,
                                           m_vis_id);
    SV_SHOW_INFO_NAMED(vis_name, marker);
    m_vis_id++;
    getchar();
#endif

    std::vector<Action> actions;
    getActions(*parent_entry, actions);

    SMPL_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        RobotState final_rstate;
        if (m_search_mode == REACHABILITY) {
            if (!checkActionPreprocessing(parent_entry->state, action, &final_rstate)) {
                continue;
            }
        }
        else {
            if (!checkAction(parent_entry->state, action, &final_rstate)) {
                continue;
            }
        }

        const WorkspaceState& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        int succ_id = createState(succ_coord);
        WorkspaceLatticeState* succ_state = getState(succ_id);
        succ_state->state = final_rstate;

        // check if this state meets the goal criteria
        const bool is_goal_succ = false;
        // const bool is_goal_succ = isGoal(final_state);

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_id);
        }

        const int edge_cost = 30;
        costs->push_back(edge_cost);

        SMPL_DEBUG_NAMED(params()->expands_log, "      succ: %d", succ_id);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(params()->expands_log, "        cost: %5d", edge_cost);
    }
}

void WorkspaceLatticeZero::ClearStates()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();
    m_state_to_id.clear();
    m_states.shrink_to_fit();
}

bool WorkspaceLatticeZero::initMotionPrimitives()
{
    m_prims.clear();

    MotionPrimitive prim;

    // create 26-connected position motions
    // for (int dx = -1; dx <= 1; ++dx) {
    //     for (int dy = -1; dy <= 1; ++dy) {
    //         for (int dz = -1; dz <= 1; ++dz) {
    //             if (dx == 0 && dy == 0 && dz == 0) {
    //                 continue;
    //             }

    //             std::vector<double> d(m_dof_count, 0.0);
    //             d[0] = m_res[0] * dx;
    //             d[1] = m_res[1] * dy;
    //             d[2] = m_res[2] * dz;
    //             prim.type = MotionPrimitive::Type::LONG_DISTANCE;
    //             prim.action.clear();
    //             prim.action.push_back(std::move(d));

    //             m_prims.push_back(prim);
    //         }
    //     }
    // }

    for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0) {
            continue;
        }
        int dy = 0, dz = 0;
        std::vector<double> d(m_dof_count, 0.0);
        d[0] = m_res[0] * dx;
        d[1] = m_res[1] * dy;
        d[2] = m_res[2] * dz;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(std::move(d));

        m_prims.push_back(prim);
    }

    for (int dy = -1; dy <= 1; ++dy) {
        if (dy == 0) {
            continue;
        }
        int dx = 0, dz = 0;
        std::vector<double> d(m_dof_count, 0.0);
        d[0] = m_res[0] * dx;
        d[1] = m_res[1] * dy;
        d[2] = m_res[2] * dz;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(std::move(d));

        m_prims.push_back(prim);
    }

    for (int dz = -1; dz <= 1; ++dz) {
        if (dz == 0) {
            continue;
        }
        int dx = 0, dy = 0;
        std::vector<double> d(m_dof_count, 0.0);
        d[0] = m_res[0] * dx;
        d[1] = m_res[1] * dy;
        d[2] = m_res[2] * dz;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(std::move(d));

        m_prims.push_back(prim);
    }

    // create 2-connected motions for rotation and free angle motions
    for (int a = 3; a < m_dof_count; ++a) {
        std::vector<double> d(m_dof_count, 0.0);

        d[a] = m_res[a] * -1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        m_prims.push_back(prim);

        d[a] = m_res[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        m_prims.push_back(prim);
    }

    return true;
}

bool WorkspaceLatticeZero::isGoal(const WorkspaceState& state) const
{
    // check position
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL: {
        // SMPL_WARN_ONCE("WorkspaceLattice joint-space goals not implemented");
        WorkspaceCoord coord;
        stateWorkspaceToCoord(state, coord);
        WorkspaceLatticeState s;
        s.coord = coord;
        auto sit = m_state_to_id.find(&s);
        int state_id;
        if (sit != m_state_to_id.end()) {
            state_id = sit->second;
        }
        return state_id == m_goal_state_id;
    }
    case GoalType::XYZ_RPY_GOAL: {
        double dx = std::fabs(state[0] - goal().tgt_off_pose.translation()[0]);
        double dy = std::fabs(state[1] - goal().tgt_off_pose.translation()[1]);
        double dz = std::fabs(state[2] - goal().tgt_off_pose.translation()[2]);
        if (dx <= goal().xyz_tolerance[0] &&
            dy <= goal().xyz_tolerance[1] &&
            dz <= goal().xyz_tolerance[2])
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                auto now = clock::now();
                double time_to_goal_region =
                        std::chrono::duration<double>(now - m_t_start).count();
                m_near_goal = true;
                SMPL_INFO("search is at the goal position after %0.3f sec", time_to_goal_region);
            }

            Eigen::Quaterniond qg(goal().tgt_off_pose.rotation());
            Eigen::Quaterniond q(
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX()));

            if (q.dot(qg) < 0.0) {
                qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
            }

//            const double theta = angles::normalize_angle(Eigen::AngleAxisd(qg.conjugate() * q).angle());
            const double theta = angles::normalize_angle(2.0 * acos(q.dot(qg)));
            if (theta < goal().rpy_tolerance[0]) {
                return true;
            }
        }
        return false;
    }   break;
    case GoalType::XYZ_GOAL: {
        SMPL_WARN_ONCE("WorkspaceLattice xyz goals not implemented");
        return false;
    }   break;
    default:
        return false;
    }
}

bool WorkspaceLatticeZero::checkActionPreprocessing(
    const RobotState& state,
    const Action& action,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    WorkspaceState end_state;
    end_state = action[action.size() - 1];

    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  end state: " << end_state);
    if (!IsWorkspaceStateInStartRegion(end_state)) {
        violation_mask |= 0x00000001;
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

bool WorkspaceLatticeZero::checkAction(
    const RobotState& state,
    const Action& action,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    // check waypoints for ik solutions and joint limits
    for (size_t widx = 0; widx < action.size(); ++widx) {
        const WorkspaceState& istate = action[widx];

        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        " << widx << ": " << istate);

        RobotState irstate;
        if (!stateWorkspaceToRobot(istate, m_ik_seed, irstate)) {
            SMPL_DEBUG_NAMED(params()->expands_log, "         -> failed to find ik solution");
            violation_mask |= 0x00000001;
            break;
        }

        wptraj.push_back(irstate);

        if (!robot()->checkJointLimits(irstate)) {
            SMPL_DEBUG_NAMED(params()->expands_log, "        -> violates joint limits");
            violation_mask |= 0x00000002;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (!collisionChecker()->isStateToStateValid(state, wptraj[0])) {
        SMPL_DEBUG_NAMED(params()->expands_log, "        -> path to first waypoint in collision");
        // SMPL_INFO_STREAM_NAMED(params()->expands_log, "  attractor coord   : " << action[0]);
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    for (size_t widx = 1; widx < wptraj.size(); ++widx) {
        const RobotState& prev_istate = wptraj[widx - 1];
        const RobotState& curr_istate = wptraj[widx];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate)) {
            SMPL_DEBUG_NAMED(params()->expands_log, "        -> path between waypoints in collision");
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    if (final_rstate) {
        *final_rstate = wptraj.back();
    }
    return true;
}

} // namespace motion
} // namespace sbpl
