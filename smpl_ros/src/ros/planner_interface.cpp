////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2009, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/ros/planner_interface.h>

// standard includes
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <utility>

// system includes
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <sbpl/planners/mhaplanner.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <smpl/angles.h>
#include <smpl/post_processing.h>
#include <smpl/time.h>
#include <smpl/types.h>

#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>

#include <smpl/graph/adaptive_workspace_lattice.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/workspace_lattice_zero.h>
#include <smpl/graph/manip_lattice_egraph.h>

#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/workspace_dist_heuristic.h>
#include <smpl/heuristic/multi_frame_bfs_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>

#include <smpl/search/adaptive_planner.h>
#include <smpl/search/arastar.h>
#include <smpl/search/arastar_zero.h>
#include <smpl/search/experience_graph_planner.h>
#include <smpl/search/awastar.h>

namespace sbpl {
namespace motion {

template <class T, class... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T> {
    return std::unique_ptr<T>(new T(args...));
}

const char* PI_LOGGER = "simple";

struct ManipLatticeActionSpaceParams
{
    std::string mprim_filename;
    bool use_multiple_ik_solutions = false;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_thresh;
    double rpy_snap_thresh;
    double xyzrpy_snap_thresh;
    double short_dist_mprims_thresh;
};

// Lookup parameters for ManipLatticeActionSpace, setting reasonable defaults
// for missing parameters. Return false if any required parameter is not found.
bool GetManipLatticeActionSpaceParams(
    ManipLatticeActionSpaceParams& params,
    const PlanningParams& pp)
{
    if (!pp.getParam("mprim_filename", params.mprim_filename)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'mprim_filename' not found in planning params");
        return false;
    }

    pp.param("use_multiple_ik_solutions", params.use_multiple_ik_solutions, false);

    pp.param("use_xyz_snap_mprim", params.use_xyz_snap_mprim, false);
    pp.param("use_rpy_snap_mprim", params.use_rpy_snap_mprim, false);
    pp.param("use_xyzrpy_snap_mprim", params.use_xyzrpy_snap_mprim, false);
    pp.param("use_short_dist_mprims", params.use_short_dist_mprims, false);

    pp.param("xyz_snap_dist_thresh", params.xyz_snap_thresh, 0.0);
    pp.param("rpy_snap_dist_thresh", params.rpy_snap_thresh, 0.0);
    pp.param("xyzrpy_snap_dist_thresh", params.xyzrpy_snap_thresh, 0.0);
    pp.param("short_dist_mprims_thresh", params.short_dist_mprims_thresh, 0.0);
    return true;
}

auto MakeManipLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return nullptr;
    }

    std::unordered_map<std::string, double> disc;
    std::stringstream ss(disc_string);
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        const std::string& vname = robot->getPlanningJoints()[vidx];
        const size_t sidx = vname.find('/');
        if (sidx != std::string::npos) {
            // adjust variable name if a variable of a multi-dof joint
            std::string mdof_vname =
                    vname.substr(0, sidx) + "_" + vname.substr(sidx + 1);
            auto dit = disc.find(mdof_vname);
            if (dit == disc.end()) {
                ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return nullptr;
            }
            resolutions[vidx] = dit->second;
        } else {
            auto dit = disc.find(vname);
            if (dit == disc.end()) {
                ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return nullptr;
            }
            resolutions[vidx] = dit->second;
        }

        ROS_DEBUG_NAMED(PI_LOGGER, "resolution(%s) = %0.3f", vname.c_str(), resolutions[vidx]);
    }

    ManipLatticeActionSpaceParams action_params;
    if (!GetManipLatticeActionSpaceParams(action_params, *params)) {
        return nullptr;
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLattice and
    // ManipLatticeActionSpace
    struct SimpleManipLattice : public ManipLattice {
        ManipLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleManipLattice>();

    if (!space->init(robot, checker, params, resolutions, &space->actions)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice");
        return nullptr;
    }

    if (!space->actions.init(space.get())) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice Action Space");
        return nullptr;
    }

    if (grid) {
        space->setVisualizationFrameId(grid->getReferenceFrame());
    }

    auto& actions = space->actions;
    actions.useMultipleIkSolutions(action_params.use_multiple_ik_solutions);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ, action_params.use_xyz_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_RPY, action_params.use_rpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.use_xyzrpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SHORT_DISTANCE, action_params.use_short_dist_mprims);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ, action_params.xyz_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_RPY, action_params.rpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.xyzrpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SHORT_DISTANCE, action_params.short_dist_mprims_thresh);

    if (!actions.load(action_params.mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return nullptr;
    }

    ROS_DEBUG_NAMED(PI_LOGGER, "Action Set:");
    for (auto ait = actions.begin(); ait != actions.end(); ++ait) {
        ROS_DEBUG_NAMED(PI_LOGGER, "  type: %s", to_cstring(ait->type));
        if (ait->type == MotionPrimitive::SNAP_TO_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_RPY));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY));
        } else if (ait->type == MotionPrimitive::LONG_DISTANCE ||
            ait->type == MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "    action: " << ait->action);
        }
    }

    return std::move(space);
}

auto MakeManipLatticeEGraph(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return nullptr;
    }
    std::map<std::string, double> disc;
    std::stringstream ss(disc_string);
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        const std::string& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == disc.end()) {
            ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return nullptr;
        }
        resolutions[vidx] = dit->second;
    }

    ManipLatticeActionSpaceParams action_params;
    if (!GetManipLatticeActionSpaceParams(action_params, *params)) {
        return nullptr; // errors logged within
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLatticeEgraph and
    // ManipLatticeActionSpace
    struct SimpleManipLatticeEgraph : public ManipLatticeEgraph {
        ManipLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleManipLatticeEgraph>();

    if (!space->init(robot, checker, params, resolutions, &space->actions)) {
        ROS_ERROR("Failed to initialize Manip Lattice Egraph");
        return nullptr;
    }

    if (!space->actions.init(space.get())) {
        ROS_ERROR("Failed to initialize Manip Lattice Action Space");
        return nullptr;
    }

    auto& actions = space->actions;
    actions.useMultipleIkSolutions(action_params.use_multiple_ik_solutions);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ, action_params.use_xyz_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_RPY, action_params.use_rpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.use_xyzrpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SHORT_DISTANCE, action_params.use_short_dist_mprims);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ, action_params.xyz_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_RPY, action_params.rpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.xyzrpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SHORT_DISTANCE, action_params.short_dist_mprims_thresh);
    if (!actions.load(action_params.mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return nullptr;
    }

    std::string egraph_path;
    if (params->getParam("egraph_path", egraph_path)) {
        // warning printed within, allow to fail silently
        (void)space->loadExperienceGraph(egraph_path);
    } else {
        ROS_WARN("No experience graph file parameter");
    }

    return std::move(space);
}

auto MakeWorkspaceLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");

    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    wsp.R_count = 72;
    wsp.P_count = 36 + 1;
    wsp.Y_count = 72;

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return nullptr;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(5.0));

    auto space = make_unique<WorkspaceLattice>();
    if (!space->init(robot, checker, params, wsp)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return nullptr;
    }

    space->setVisualizationFrameId(grid->getReferenceFrame());

    return std::move(space);
}

auto MakeWorkspaceLatticeZero(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice Zero");

    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    wsp.R_count = 72;
    wsp.P_count = 36 + 1;
    wsp.Y_count = 72;

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice Zero requires Redundant Manipulator Interface");
        return nullptr;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(5.0));

    auto space = make_unique<WorkspaceLatticeZero>();
    if (!space->init(robot, checker, params, wsp)) {
        ROS_ERROR("Failed to initialize Workspace Lattice Zero");
        return nullptr;
    }

    space->setVisualizationFrameId(grid->getReferenceFrame());

    return std::move(space);
}

auto MakeAdaptiveWorkspaceLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");
    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    wsp.R_count = 36; //360;
    wsp.P_count = 19; //180 + 1;
    wsp.Y_count = 36; //360;

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return nullptr;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(1.0));

    auto space = make_unique<AdaptiveWorkspaceLattice>();
    if (!space->init(robot, checker, params, wsp, grid)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return nullptr;
    }

    return std::move(space);
}

auto MakeMultiFrameBFSHeuristic(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<MultiFrameBfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }
    return std::move(h);
}

auto MakeBFSHeuristic(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<BfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }
    return std::move(h);
};

auto MakeEuclidDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<EuclidDistHeuristic>();

    if (!h->init(space)) {
        return nullptr;
    }

    double wx, wy, wz, wr;
    params.param("x_coeff", wx, 1.0);
    params.param("y_coeff", wy, 1.0);
    params.param("z_coeff", wz, 1.0);
    params.param("rot_coeff", wr, 1.0);

    h->setWeightX(wx);
    h->setWeightY(wx);
    h->setWeightZ(wx);
    h->setWeightRot(wx);
    return std::move(h);
};

auto MakeWorkspaceDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<WorkspaceDistHeuristic>();

    if (!h->init(space)) {
        return nullptr;
    }

    double wx, wy, wz, wr, wfa;
    params.param("x_coeff", wx, 1.0);
    params.param("y_coeff", wy, 1.0);
    params.param("z_coeff", wz, 1.0);
    params.param("rot_coeff", wr, 1.0);
    params.param("fa_coeff", wfa, 1.0);

    h->setWeightX(wx);
    h->setWeightY(wy);
    h->setWeightZ(wz);
    h->setWeightRot(wr);
    h->setWeightFreeAngles(wfa);
    return std::move(h);
};

auto MakeJointDistHeuristic(RobotPlanningSpace* space)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<JointDistHeuristic>();
    if (!h->init(space)) {
        return nullptr;
    }
    return std::move(h);
};

auto MakeDijkstraEgraphHeuristic3D(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<DijkstraEgraphHeuristic3D>();

//    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }

    return std::move(h);
};

auto MakeJointDistEGraphHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    struct JointDistEGraphHeuristic : public GenericEgraphHeuristic {
        JointDistHeuristic jd;
    };

    auto h = make_unique<JointDistEGraphHeuristic>();
    if (!h->init(space, &h->jd)) {
        return nullptr;
    }

    double egw;
    params.param("egraph_epsilon", egw, 1.0);
    h->setWeightEGraph(egw);
    return std::move(h);
};

auto MakeARAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    const bool forward_search = true;
    auto search = make_unique<ARAStar>(space, heuristic);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (space->params()->getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double target_eps;
    if (space->params()->getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (space->params()->getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (space->params()->getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (space->params()->getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }

    double repair_time;
    if (space->params()->getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }

    return std::move(search);
}

auto MakeARAStarZero(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    const bool forward_search = true;
    auto search = make_unique<ARAStarZero>(space, heuristic);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (space->params()->getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double target_eps;
    if (space->params()->getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (space->params()->getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (space->params()->getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (space->params()->getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }

    double repair_time;
    if (space->params()->getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }

    return std::move(search);
}

auto MakeAWAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<AWAStar>(space, heuristic);
    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);
    return std::move(search);
}

auto MakeMHAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    struct MHAPlannerAdapter : public MHAPlanner {
        std::vector<Heuristic*> heuristics;

        MHAPlannerAdapter(
            DiscreteSpaceInformation* space,
            Heuristic* anchor,
            Heuristic** heurs,
            int hcount)
        :
            MHAPlanner(space, anchor, heurs, hcount)
        { }
    };

    std::vector<Heuristic*> heuristics;
    heuristics.push_back(heuristic);

    const bool forward_search = true;
    auto search = make_unique<MHAPlannerAdapter>(
            space, heuristics[0], &heuristics[0], heuristics.size());

    search->heuristics = std::move(heuristics);

    double mha_eps;
    space->params()->param("epsilon_mha", mha_eps, 1.0);
    search->set_initial_mha_eps(mha_eps);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    return std::move(search);
}

auto MakeLARAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    const bool forward_search = true;
    auto search = make_unique<LazyARAPlanner>(space, forward_search);
    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);
    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);
    return std::move(search);
}

auto MakeEGWAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    return make_unique<ExperienceGraphPlanner>(space, heuristic);
}

auto MakePADAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<AdaptivePlanner>(space, heuristic);

    double epsilon_plan;
    space->params()->param("epsilon_plan", epsilon_plan, 1.0);
    search->set_plan_eps(epsilon_plan);

    double epsilon_track;
    space->params()->param("epsilon_track", epsilon_track, 1.0);
    search->set_track_eps(epsilon_track);

    AdaptivePlanner::TimeParameters tparams;
    tparams.planning.bounded = true;
    tparams.planning.improve = false;
    tparams.planning.type = ARAStar::TimeParameters::TIME;
    tparams.planning.max_allowed_time_init = clock::duration::zero();
    tparams.planning.max_allowed_time = clock::duration::zero();

    tparams.tracking.bounded = true;
    tparams.tracking.improve = false;
    tparams.tracking.type = ARAStar::TimeParameters::TIME;
    tparams.tracking.max_allowed_time_init = std::chrono::seconds(5);
    tparams.tracking.max_allowed_time = clock::duration::zero();

    search->set_time_parameters(tparams);

    return std::move(search);
}

PlannerInterface::PlannerInterface(
    RobotModel* robot,
    CollisionChecker* checker,
    OccupancyGrid* grid)
:
    m_robot(robot),
    m_checker(checker),
    m_grid(grid),
    m_fk_iface(nullptr),
    m_params(),
    m_initialized(false),
    m_pspace(),
    m_heuristics(),
    m_planner(),
    m_sol_cost(INFINITECOST),
    m_planner_id(),
    m_req(),
    m_res()
{
    if (m_robot) {
        m_fk_iface = m_robot->getExtension<ForwardKinematicsInterface>();
    }

    ////////////////////////////////////
    // Setup Planning Space Factories //
    ////////////////////////////////////

    m_space_factories["manip"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeManipLattice(m_grid, r, c, p);
    };

    m_space_factories["manip_lattice_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeManipLatticeEGraph(m_grid, r, c, p);
    };

    m_space_factories["workspace"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeWorkspaceLattice(m_grid, r, c, p);
    };

    m_space_factories["workspace_zero"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeWorkspaceLatticeZero(m_grid, r, c, p);
    };

    m_space_factories["adaptive_workspace_lattice"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeAdaptiveWorkspaceLattice(m_grid, r, c, p);
    };

    ///////////////////////////////
    // Setup Heuristic Factories //
    ///////////////////////////////

    m_heuristic_factories["mfbfs"] = [this](RobotPlanningSpace* space) {
        return MakeMultiFrameBFSHeuristic(space, m_grid, m_params);
    };

    m_heuristic_factories["bfs"] = [this](RobotPlanningSpace* space) {
        return MakeBFSHeuristic(space, m_grid, m_params);
    };

    m_heuristic_factories["euclid"] = [this](RobotPlanningSpace* space) {
        return MakeEuclidDistHeuristic(space, m_params);
    };

    m_heuristic_factories["workspace_distance"] = [this](RobotPlanningSpace* space) {
        return MakeWorkspaceDistHeuristic(space, m_params);
    };

    m_heuristic_factories["joint_distance"] = [this](RobotPlanningSpace* space) {
        return MakeJointDistHeuristic(space);
    };

    m_heuristic_factories["bfs_egraph"] = [this](RobotPlanningSpace* space) {
        return MakeDijkstraEgraphHeuristic3D(space, m_grid, m_params);
    };

    m_heuristic_factories["joint_distance_egraph"] = [this](RobotPlanningSpace* space) {
        return MakeJointDistEGraphHeuristic(space, m_params);
    };

    /////////////////////////////
    // Setup Planner Factories //
    /////////////////////////////

    m_planner_factories["arastar"] = MakeARAStar;
    m_planner_factories["arastar_zero"] = MakeARAStarZero;
    m_planner_factories["awastar"] = MakeAWAStar;
    m_planner_factories["mhastar"] = MakeMHAStar;
    m_planner_factories["larastar"] = MakeLARAStar;
    m_planner_factories["egwastar"] = MakeEGWAStar;
    m_planner_factories["padastar"] = MakePADAStar;
}

PlannerInterface::~PlannerInterface()
{
}

bool PlannerInterface::init(const PlanningParams& params)
{
    ROS_INFO_NAMED(PI_LOGGER, "initialize arm planner interface");

    ROS_INFO_NAMED(PI_LOGGER, "  Planning Frame: %s", params.planning_frame.c_str());

    ROS_INFO_NAMED(PI_LOGGER, "  Planning Link Sphere Radius: %0.3f", params.planning_link_sphere_radius);

    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Path: %s", params.shortcut_path ? "true" : "false");
    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Type: %s", to_string(params.shortcut_type).c_str());
    ROS_INFO_NAMED(PI_LOGGER, "  Interpolate Path: %s", params.interpolate_path ? "true" : "false");

    if (!checkConstructionArgs()) {
        return false;
    }

    if (!checkParams(params)) {
        return false;
    }

    m_params = params;

    m_grid->setReferenceFrame(m_params.planning_frame);

    m_initialized = true;

    ROS_INFO_NAMED(PI_LOGGER, "initialized arm planner interface");
    return m_initialized;
}

bool PlannerInterface::checkConstructionArgs() const
{
    if (!m_robot) {
        ROS_ERROR("Robot Model given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_checker) {
        ROS_ERROR("Collision Checker given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_grid) {
        ROS_ERROR("Occupancy Grid given to Arm Planner Interface must be non-null");
        return false;
    }

    return true;
}

bool PlannerInterface::solve(
    // TODO: this planning scene is probably not being used in any meaningful way
    const moveit_msgs::PlanningScene& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    clearMotionPlanResponse(req, res);

    if (!m_initialized) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!canServiceRequest(req, res)) {
        return false;
    }

    m_req = req; // record the last attempted request

    if (req.goal_constraints.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    // TODO: lazily reinitialize planner when algorithm changes
    if (!reinitPlanner(req.planner_id)) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    // plan
    res.trajectory_start = planning_scene.robot_state;
    ROS_INFO_NAMED(PI_LOGGER, "Allowed Time (s): %0.3f", req.allowed_planning_time);

    auto then = clock::now();

    std::vector<RobotState> path;
    if (req.goal_constraints.front().position_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to position!");
        if (!planToPose(req, path, res)) {
            auto now = clock::now();
            res.planning_time = to_seconds(now - then);
            return false;
        }
    } else if (req.goal_constraints.front().joint_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to joint configuration!");
        if (!planToConfiguration(req, path, res)) {
            auto now = clock::now();
            res.planning_time = to_seconds(now - then);
            return false;
        }
    } else {
        ROS_ERROR("Both position and joint constraints empty!");
        auto now = clock::now();
        res.planning_time = to_seconds(now - then);
        return false;
    }

    ROS_DEBUG_NAMED(PI_LOGGER, "planner path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        const auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }

    postProcessPath(path);
    SV_SHOW_INFO_NAMED("trajectory", makePathVisualization(path));

    ROS_DEBUG_NAMED(PI_LOGGER, "smoothed path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        const auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }

    auto& traj = res.trajectory.joint_trajectory;
    convertJointVariablePathToJointTrajectory(path, traj);
    traj.header.seq = 0;
    traj.header.stamp = ros::Time::now();

    if (!m_params.plan_output_dir.empty()) {
        writePath(res.trajectory_start, res.trajectory);
    }

    profilePath(traj);
//    removeZeroDurationSegments(traj);

    auto now = clock::now();
    res.planning_time = to_seconds(now - then);
    m_res = res; // record the last result
    return true;
}

bool PlannerInterface::solveZero(
    // TODO: this planning scene is probably not being used in any meaningful way
    const moveit_msgs::PlanningScene& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res,
    bool query)
{
    ROS_INFO("Solve Zero");

    clearMotionPlanResponse(req, res);

    if (!m_initialized) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!canServiceRequest(req, res)) {
        return false;
    }

    m_req = req; // record the last attempted request

    if (req.goal_constraints.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    // make spaces
    auto psait = m_space_factories.find("manip");
    auto manip_space = psait->second(m_robot, m_checker, &m_params);
    if (!manip_space) {
        ROS_ERROR("Failed to build manip space");
        return false;
    }

    psait = m_space_factories.find("workspace_zero");
    auto task_space = psait->second(m_robot, m_checker, &m_params);
    if (!task_space) {
        ROS_ERROR("Failed to build manip space");
        return false;
    }

    // make heuristics
    auto hait = m_heuristic_factories.find("bfs");
    auto bfs_heuristic = hait->second(manip_space.get());
    if (!bfs_heuristic) {
        ROS_ERROR("Failed to build bfs heuristic");
        return false;
    }

    hait = m_heuristic_factories.find("workspace_distance");
    auto jd_heuristic = hait->second(task_space.get());
    if (!jd_heuristic) {
        ROS_ERROR("Failed to build wd heuristic");
        return false;
    }

    //insert jd heuristic in the robot planning space
    if (!task_space->insertHeuristic(jd_heuristic.get())) {
        ROS_ERROR("Could not insert heuristic");
        return false;
    }

    // make planners
    auto pait = m_planner_factories.find("arastar");
    if (pait == m_planner_factories.end()) {
        ROS_ERROR("Unrecognized search name arastar");
        return false;
    }
    auto planner1 = pait->second(manip_space.get(), bfs_heuristic.get());

    pait = m_planner_factories.find("arastar_zero");
    if (pait == m_planner_factories.end()) {
        ROS_ERROR("Unrecognized search name arastar");
        return false;
    }
    auto planner2 = pait->second(task_space.get(), jd_heuristic.get());

    // fill start and goal
    RobotState initial_positions;
    if (!fillStartState(m_req.start_state, initial_positions)) {
        ROS_ERROR("Failed to fill start state");
        return false;
    }

    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());
    const auto& goal_constraints = goal_constraints_v.front();

    GoalConstraint goal;
    if (req.goal_constraints.front().position_constraints.size() > 0) {
        if (!fillGoalPositionConstraint(goal_constraints, goal)) {
            ROS_ERROR("Failed to fill goal position constraint");
            return false;
        }
    }
    else if (req.goal_constraints.front().joint_constraints.size() > 0) {
        if (!fillGoalConfigurationConstraint(goal_constraints, goal)) {
            ROS_ERROR("Failed to fill goal configuration constraint");
            return false;
        }
    } else {
        ROS_ERROR("Both position and joint constraints empty!");
    }

    m_zero_planner.reset(new ZeroTimePlanner(
        initial_positions,
        goal,
        dynamic_cast<ManipLattice*>(manip_space.get()),
        dynamic_cast<WorkspaceLatticeZero*>(task_space.get()),
        dynamic_cast<ARAStar*>(planner1.get()),
        dynamic_cast<ARAStarZero*>(planner2.get())));

    auto then = clock::now();

    std::vector<RobotState> path;

    if (!query) {
        ROS_INFO("Preprocessing Start Region");
        m_zero_planner->PreProcess();
    }
    else {
        // int num_queries = 10;
        // ROS_INFO("Going to run %d random queries", num_queries);
        // for (int i = 0; i < num_queries; ++i) {
        // ROS_INFO("\n************* QUERY %d ***************", i);
        ROS_INFO("Zero time query");
        m_zero_planner->Query(path);
        postProcessPath(path);
        SV_SHOW_INFO_NAMED("trajectory", makePathVisualization(path));

        ROS_DEBUG_NAMED(PI_LOGGER, "smoothed path:");
        for (size_t pidx = 0; pidx < path.size(); ++pidx) {
            const auto& point = path[pidx];
            ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
        }

        auto& traj = res.trajectory.joint_trajectory;
        convertJointVariablePathToJointTrajectory(path, traj);
        traj.header.seq = 0;
        traj.header.stamp = ros::Time::now();
    
        if (!m_params.plan_output_dir.empty()) {
            writePath(res.trajectory_start, res.trajectory);
        }

        profilePath(traj);
    //    removeZeroDurationSegments(traj);

        auto now = clock::now();
        res.planning_time = to_seconds(now - then);
        m_res = res; // record the last result
        // getchar();
    }
    // }
    bfs_heuristic.release();    //avoid crash

    return true;
}

bool PlannerInterface::checkParams(
    const PlanningParams& params) const
{
    if (params.planning_frame.empty()) {
        return false;
    }

    // TODO: check for existence of planning joints in robot model

    if (params.cost_per_cell < 0) {
        return false;
    }

    return true;
}

bool PlannerInterface::setStart(const moveit_msgs::RobotState& state)
{
    ROS_INFO_NAMED(PI_LOGGER, "set start configuration");

    RobotState initial_positions;

    if (!fillStartState(m_req.start_state, initial_positions)) {
        ROS_ERROR("Failed to fill start state");
        return false;
    }

    ROS_INFO_STREAM_NAMED(PI_LOGGER, "  joint variables: " << initial_positions);

    if (!m_pspace->setStart(initial_positions)) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    const int start_id = m_pspace->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set");
        return false;
    }

    if (m_planner->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalConfiguration(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Set goal configuration");

    GoalConstraint goal;
    if (!fillGoalConfigurationConstraint(goal_constraints, goal)) {
        ROS_ERROR("Failed to fill goal configuration constraint");
        return false;
    }

    // TODO: really need to reevaluate the necessity of the planning link
    if (m_fk_iface) {
        goal.pose = m_fk_iface->computeFK(goal.angles);
        goal.tgt_off_pose = goal.pose;
    } else {
        goal.pose = goal.tgt_off_pose = Eigen::Affine3d::Identity();
    }

    // set sbpl environment goal
    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }

    // set planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalPosition(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Setting goal position");

    GoalConstraint goal;
    if (!fillGoalPositionConstraint(goal_constraints, goal)) {
        ROS_ERROR("Failed to fill goal position constraint");
        return false;
    }

    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }

    // set sbpl planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

bool PlannerInterface::plan(double allowed_time, std::vector<RobotState>& path)
{
    // NOTE: this should be done after setting the start/goal in the environment
    // to allow the heuristic to tailor the visualization to the current
    // scenario
    SV_SHOW_DEBUG_NAMED("bfs_walls", getBfsWallsVisualization());
    SV_SHOW_DEBUG_NAMED("bfs_values", getBfsValuesVisualization());

    ROS_WARN_NAMED(PI_LOGGER, "Planning!!!!!");
    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner->force_planning_from_scratch();

    // plan
    b_ret = m_planner->replan(allowed_time, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER, "Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning succeeded");
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Final): %d", m_planner->get_n_expands());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_INFO_NAMED(PI_LOGGER, "  Solution Cost: %d", m_sol_cost);

        path.clear();
        if (!m_pspace->extractPath(solution_state_ids, path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
    }
    return b_ret;
}

bool PlannerInterface::planToPose(
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // transform goal pose into reference_frame

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    if (!setGoalPosition(goal_constraints)) {
        ROS_ERROR("Failed to set goal position");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(req.allowed_planning_time, path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

bool PlannerInterface::planToConfiguration(
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    if (!setGoalConfiguration(goal_constraints)) {
        ROS_ERROR("Failed to set goal position");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(req.allowed_planning_time, path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

/// Test if a particular set of goal constraints it supported.
///
/// This tests whether, in general, any planning algorithm supported by this
/// interface can support a particular set of constraints. Certain planning
/// algorithms may not be able to handle a given set of constraints. This
/// method also cannot check for validity of constraints against a particular
/// robot model at this step. In particular, it cannot assert that the a set
/// of joint constraints lists a constraint for each joint, which is currently
/// required.
bool PlannerInterface::SupportsGoalConstraints(
    const std::vector<moveit_msgs::Constraints>& constraints,
    std::string& why)
{
    if (constraints.empty()) {
        return true;
    }

    if (constraints.size() > 1) {
        why = "no planner currently supports more than one goal constraint";
        return false;
    }

    const moveit_msgs::Constraints& constraint = constraints.front();

    if (!constraint.visibility_constraints.empty()) {
        why = "no planner currently supports goal visibility constraints";
        return false;
    }

    // technically multiple goal position/orientation constraints can be
    // solved for if there is one position/orientation volume that entirely
    // bounds all other position/orientation volumes...ignoring for now

    if (constraint.position_constraints.size() > 1) {
        why = "no planner currently supports more than one position constraint";
        return false;
    }

    if (constraint.orientation_constraints.size() > 1) {
        why = "no planner currently supports more than one orientation constraint";
        return false;
    }

    const bool no_pose_constraint =
            constraint.position_constraints.empty() &&
            constraint.orientation_constraints.empty();
    const bool has_pose_constraint =
            constraint.position_constraints.size() == 1 &&
            constraint.orientation_constraints.size() == 1;
    const bool has_joint_constraints = !constraint.joint_constraints.empty();

    if (has_joint_constraints) {
        if (has_pose_constraint) {
            why = "no planner currently supports both pose and joint constraints";
            return false;
        }
    } else {
        if (no_pose_constraint) {
            // no constraints -> ok!
            return true;
        } else if (has_pose_constraint) {
            if (constraint.position_constraints.front().link_name !=
                constraint.orientation_constraints.front().link_name)
            {
                why = "pose constraint must be for a single link";
                return false;
            }
            return true;
        } else {
            // pose constraint is invalid
            why = "no planner supports only one position constraint or one orientation constraint";
            return false;
        }
    }

    // made it through the gauntlet
    return true;
}

bool PlannerInterface::canServiceRequest(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    // check for an empty start state
    // TODO: generalize this to "missing necessary state information"
    if (req.start_state.joint_state.position.empty()) {
        ROS_ERROR("No start state given. Unable to plan.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    // check if position & orientation constraints is empty
    const moveit_msgs::Constraints& goal_constraints =
            req.goal_constraints.front();

    if ((
            goal_constraints.position_constraints.empty() ||
            goal_constraints.orientation_constraints.empty()
        ) &&
        goal_constraints.joint_constraints.empty())
    {
        ROS_ERROR("Position or orientation constraint is empty");
        ROS_ERROR("Joint constraint is empty");
        ROS_ERROR("PlannerInterface expects a 6D pose constraint or set of joint constraints");
        return false;
    }

    // check if there is more than one goal constraint
    if (goal_constraints.position_constraints.size() > 1 ||
        goal_constraints.orientation_constraints.size() > 1)
    {
        ROS_WARN_NAMED(PI_LOGGER, "The planning request message contains %zd position and %zd orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", goal_constraints.position_constraints.size(), goal_constraints.orientation_constraints.size());
    }

    return true;
}

std::map<std::string, double> PlannerInterface::getPlannerStats()
{
    std::map<std::string, double> stats;
    stats["initial solution planning time"] = m_planner->get_initial_eps_planning_time();
    stats["initial epsilon"] = m_planner->get_initial_eps();
    stats["initial solution expansions"] = m_planner->get_n_expands_init_solution();
    stats["final epsilon planning time"] = m_planner->get_final_eps_planning_time();
    stats["final epsilon"] = m_planner->get_final_epsilon();
    stats["solution epsilon"] = m_planner->get_solution_eps();
    stats["expansions"] = m_planner->get_n_expands();
    stats["solution cost"] = m_sol_cost;
    return stats;
}

auto PlannerInterface::makePathVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }

    double cinc = 1.0 / double(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = m_checker->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            const float r = 0.1f;
            const float g = cinc * (float)(path.size() - (i + 1));
            const float b = cinc * (float)i;
            marker.color = visual::Color{ r, g, b, 1.0f };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = "trajectory";
        marker.id = i;
    }

    return ma;
}

auto PlannerInterface::getBfsValuesVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getValuesVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getValuesVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getValuesVisualization();
    } else {
        return visual::Marker{ };
    }
}

auto PlannerInterface::getBfsWallsVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getWallsVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getWallsVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getWallsVisualization();
    } else {
        return visual::Marker{ };
    }
}

bool PlannerInterface::fillStartState(
    const moveit_msgs::RobotState& state,
    RobotState& start_state)
{
    if (!state.multi_dof_joint_state.joint_names.empty()) {
        const auto& mdof_joint_names = state.multi_dof_joint_state.joint_names;
        for (const std::string& joint_name : m_robot->getPlanningJoints()) {
            auto it = std::find(mdof_joint_names.begin(), mdof_joint_names.end(), joint_name);
            if (it != mdof_joint_names.end()) {
                ROS_WARN_NAMED(PI_LOGGER, "planner does not currently support planning for multi-dof joints. found '%s' in planning joints", joint_name.c_str());
            }
        }
    }

    // RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state.joint_state,
            state.multi_dof_joint_state,
            m_robot->getPlanningJoints(),
            start_state,
            missing))
    {
        ROS_ERROR("start state is missing planning joints: ");
        return false;
    }
    return true;
}

bool PlannerInterface::fillGoalConfigurationConstraint(
    const moveit_msgs::Constraints& goal_constraints,
    GoalConstraint& goal)
{
    std::vector<double> sbpl_angle_goal(m_robot->jointVariableCount(), 0);
    std::vector<double> sbpl_angle_tolerance(m_robot->jointVariableCount(), angles::to_radians(3.0));

    if (goal_constraints.joint_constraints.size() < m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "All %zu arm joint constraints must be specified for goal!", m_robot->jointVariableCount());
        return false;
    }
    if (goal_constraints.joint_constraints.size() > m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "%d joint constraints specified! Using the first %zu!", (int)goal_constraints.joint_constraints.size(), m_robot->jointVariableCount());
        return false;
    }

    const size_t num_angle_constraints = std::min(
            goal_constraints.joint_constraints.size(), sbpl_angle_goal.size());
    for (size_t i = 0; i < num_angle_constraints; i++) {
        const auto& joint_constraint = goal_constraints.joint_constraints[i];
        const std::string& joint_name = joint_constraint.joint_name;
        auto jit = std::find(
                m_robot->getPlanningJoints().begin(),
                m_robot->getPlanningJoints().end(),
                joint_name);
        if (jit == m_robot->getPlanningJoints().end()) {
            ROS_ERROR("Failed to find goal constraint for joint '%s'", joint_name.c_str());
            return false;
        }
        int jidx = std::distance(m_robot->getPlanningJoints().begin(), jit);
        sbpl_angle_goal[jidx] = joint_constraint.position;
        sbpl_angle_tolerance[jidx] = std::min(
                fabs(joint_constraint.tolerance_above),
                fabs(joint_constraint.tolerance_below));
        ROS_INFO_NAMED(PI_LOGGER, "Joint %zu [%s]: goal position: %.3f, goal tolerance: %.3f", i, joint_name.c_str(), sbpl_angle_goal[jidx], sbpl_angle_tolerance[jidx]);
    }

    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = sbpl_angle_goal;
    goal.angle_tolerances = sbpl_angle_tolerance;
}

bool PlannerInterface::fillGoalPositionConstraint(
    const moveit_msgs::Constraints& goal_constraints,
    GoalConstraint& goal)
{
    Eigen::Affine3d goal_pose;
    Eigen::Vector3d offset;
    if (!extractGoalPoseFromGoalConstraints(
            goal_constraints, goal_pose, offset))
    {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal pose from goal constraints");
        return false;
    }

    goal.type = GoalType::XYZ_RPY_GOAL;
    goal.pose = goal_pose;
    goal.xyz_offset[0] = offset.x();
    goal.xyz_offset[1] = offset.y();
    goal.xyz_offset[2] = offset.z();

    std::vector<double> sbpl_tolerance(6, 0.0);
    if (!extractGoalToleranceFromGoalConstraints(goal_constraints, &sbpl_tolerance[0])) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }

    goal.xyz_tolerance[0] = sbpl_tolerance[0];
    goal.xyz_tolerance[1] = sbpl_tolerance[1];
    goal.xyz_tolerance[2] = sbpl_tolerance[2];
    goal.rpy_tolerance[0] = sbpl_tolerance[3];
    goal.rpy_tolerance[1] = sbpl_tolerance[4];
    goal.rpy_tolerance[2] = sbpl_tolerance[5];

    ROS_INFO_NAMED(PI_LOGGER, "New Goal");
    ROS_INFO_NAMED(PI_LOGGER, "    frame: %s", m_params.planning_frame.c_str());
    double yaw, pitch, roll;
    angles::get_euler_zyx(goal.pose.rotation(), yaw, pitch, roll);
    ROS_INFO_NAMED(PI_LOGGER, "    pose: (x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f)", goal.pose.translation()[0], goal.pose.translation()[1], goal.pose.translation()[2], yaw, pitch, roll);
    ROS_INFO_NAMED(PI_LOGGER, "    offset: (%0.3f, %0.3f, %0.3f)", goal.xyz_offset[0], goal.xyz_offset[1], goal.xyz_offset[2]);
    ROS_INFO_NAMED(PI_LOGGER, "    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)", sbpl_tolerance[0], sbpl_tolerance[1], sbpl_tolerance[2], sbpl_tolerance[3], sbpl_tolerance[4], sbpl_tolerance[5]);

    // ...a lot more relies on this than I had hoped
    Eigen::Affine3d target_pose = goal_pose * Eigen::Translation3d(offset);
    goal.tgt_off_pose = target_pose;
    return true;
}

bool PlannerInterface::extractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    Eigen::Affine3d& goal_pose,
    Eigen::Vector3d& offset) const
{
    if (constraints.position_constraints.empty() ||
        constraints.orientation_constraints.empty())
    {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one position and one orientation constraint");
        return false;
    }

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?
    const moveit_msgs::PositionConstraint& position_constraint = constraints.position_constraints.front();
    const moveit_msgs::OrientationConstraint& orientation_constraint = constraints.orientation_constraints.front();

    if (position_constraint.constraint_region.primitive_poses.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    const shape_msgs::SolidPrimitive& bounding_primitive = position_constraint.constraint_region.primitives.front();
    const geometry_msgs::Pose& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Affine3d T_planning_eef = // T_planning_off * T_off_eef;
            Eigen::Translation3d(
                    primitive_pose.position.x,
                    primitive_pose.position.y,
                    primitive_pose.position.z) *
            Eigen::Quaterniond(
                    primitive_pose.orientation.w,
                    primitive_pose.orientation.x,
                    primitive_pose.orientation.y,
                    primitive_pose.orientation.z);
    Eigen::Vector3d eef_pos(T_planning_eef.translation());

    Eigen::Quaterniond eef_orientation;
    tf::quaternionMsgToEigen(orientation_constraint.orientation, eef_orientation);

    goal_pose = Eigen::Translation3d(eef_pos) * eef_orientation;

    tf::vectorMsgToEigen(position_constraint.target_point_offset, offset);
    return true;
}

bool PlannerInterface::extractGoalToleranceFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    double* tol)
{
    if (!goal_constraints.position_constraints.empty() &&
        !goal_constraints.position_constraints.front()
                .constraint_region.primitives.empty())
    {
        const moveit_msgs::PositionConstraint& position_constraint =
                goal_constraints.position_constraints.front();
        const shape_msgs::SolidPrimitive& constraint_primitive =
                position_constraint.constraint_region.primitives.front();
        const std::vector<double>& dims = constraint_primitive.dimensions;
        switch (constraint_primitive.type) {
        case shape_msgs::SolidPrimitive::BOX:
            tol[0] = dims[shape_msgs::SolidPrimitive::BOX_X];
            tol[1] = dims[shape_msgs::SolidPrimitive::BOX_Y];
            tol[2] = dims[shape_msgs::SolidPrimitive::BOX_Z];
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            tol[0] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            tol[0] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CONE:
            tol[0] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            break;
        }
    }
    else {
        tol[0] = tol[1] = tol[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        const std::vector<moveit_msgs::OrientationConstraint>& orientation_constraints = goal_constraints.orientation_constraints;
        const moveit_msgs::OrientationConstraint& orientation_constraint = orientation_constraints.front();
        tol[3] = orientation_constraint.absolute_x_axis_tolerance;
        tol[4] = orientation_constraint.absolute_y_axis_tolerance;
        tol[5] = orientation_constraint.absolute_z_axis_tolerance;
    }
    else {
        tol[3] = tol[4] = tol[5] = 0.0;
    }
    return true;
}

void PlannerInterface::clearMotionPlanResponse(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
//    res.trajectory_start.joint_state;
//    res.trajectory_start.multi_dof_joint_state;
//    res.trajectory_start.attached_collision_objects;
    res.trajectory_start.is_diff = false;
    res.group_name = req.group_name;
    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.joint_trajectory.header.frame_id = "";
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.multi_dof_joint_trajectory.header.seq = 0;
    res.trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.multi_dof_joint_trajectory.header.frame_id = "";
    res.trajectory.multi_dof_joint_trajectory.joint_names.clear();
    res.trajectory.multi_dof_joint_trajectory.points.clear();
    res.planning_time = 0.0;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
}

bool PlannerInterface::parsePlannerID(
    const std::string& planner_id,
    std::string& space_name,
    std::string& heuristic_name,
    std::string& search_name) const
{
    boost::regex alg_regex("(\\w+)(?:\\.(\\w+))?(?:\\.(\\w+))?");

    boost::smatch sm;

    ROS_INFO("Match planner id '%s' against regex '%s'", planner_id.c_str(), alg_regex.str().c_str());
    if (!boost::regex_match(planner_id, sm, alg_regex)) {
        return false;
    }

    const std::string default_search_name = "arastar";
    const std::string default_heuristic_name = "bfs";
    const std::string default_space_name = "manip";

    if (sm.size() < 2 || sm[1].str().empty()) {
        search_name = default_search_name;
    } else {
        search_name = sm[1];
    }

    if (sm.size() < 3 || sm[2].str().empty()) {
        heuristic_name = default_heuristic_name;
    } else {
        heuristic_name = sm[2];
    }

    if (sm.size() < 4 || sm[3].str().empty()) {
        space_name = default_space_name;
    } else {
        space_name = sm[3];
    }

    return true;
}

void PlannerInterface::clearGraphStateToPlannerStateMap()
{
    if (!m_pspace) {
        return;
    }

    std::vector<int*>& state_id_to_index = m_pspace->StateID2IndexMapping;
    for (int* mapping : state_id_to_index) {
        for (int i = 0; i < NUMOFINDICES_STATEID2IND; ++i) {
            mapping[i] = -1;
        }
    }
}

bool PlannerInterface::reinitPlanner(const std::string& planner_id)
{
    if (planner_id == m_planner_id) {
        // TODO: check for specification of default planning components when
        // they may not have been previously specified
        return true;
    }

    ROS_INFO_NAMED(PI_LOGGER, "Initialize planner");

    std::string search_name;
    std::string heuristic_name;
    std::string space_name;
    if (!parsePlannerID(planner_id, space_name, heuristic_name, search_name)) {
        ROS_ERROR("Failed to parse planner setup");
        return false;
    }

    ROS_INFO_NAMED(PI_LOGGER, " -> Planning Space: %s", space_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Heuristic: %s", heuristic_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Search: %s", search_name.c_str());

    auto psait = m_space_factories.find(space_name);
    if (psait == m_space_factories.end()) {
        ROS_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
        return false;
    }

    m_pspace = psait->second(m_robot, m_checker, &m_params);
    if (!m_pspace) {
        ROS_ERROR("Failed to build planning space '%s'", space_name.c_str());
        return false;
    }

    auto hait = m_heuristic_factories.find(heuristic_name);
    if (hait == m_heuristic_factories.end()) {
        ROS_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
        return false;
    }

    auto heuristic = hait->second(m_pspace.get());
    if (!heuristic) {
        ROS_ERROR("Failed to build heuristic '%s'", heuristic_name.c_str());
        return false;
    }

    // initialize heuristics
    m_heuristics.clear();
    m_heuristics.insert(std::make_pair(heuristic_name, std::move(heuristic)));

    for (const auto& entry : m_heuristics) {
        m_pspace->insertHeuristic(entry.second.get());
    }

    auto pait = m_planner_factories.find(search_name);
    if (pait == m_planner_factories.end()) {
        ROS_ERROR("Unrecognized search name '%s'", search_name.c_str());
        return false;
    }

    auto first_heuristic = begin(m_heuristics);
    m_planner = pait->second(m_pspace.get(), first_heuristic->second.get());
    if (!m_planner) {
        ROS_ERROR("Failed to build planner '%s'", search_name.c_str());
        return false;
    }
    m_planner_id = planner_id;
    return true;
}

void PlannerInterface::profilePath(trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    auto& joint_names = traj.joint_names;

    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev_point = traj.points[i - 1];
        auto& curr_point = traj.points[i];

        // find the maximum time required for any joint to reach the next
        // waypoint
        double max_time = 0.0;
        for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
            const double from_pos = prev_point.positions[jidx];
            const double to_pos = curr_point.positions[jidx];
            const double vel = m_robot->velLimit(jidx);
            if (vel <= 0.0) {
                continue;
            }
            double t = 0.0;
            if (m_robot->isContinuous(jidx)) {
                t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
            } else {
                t = fabs(to_pos - from_pos) / vel;
            }

            max_time = std::max(max_time, t);
        }

        curr_point.time_from_start = prev_point.time_from_start + ros::Duration(max_time);
    }
}

void PlannerInterface::removeZeroDurationSegments(
    trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    // filter out any duplicate points
    // TODO: find out where these are happening
    size_t end_idx = 1; // current end of the non-filtered range
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev = traj.points[end_idx - 1];
        auto& curr = traj.points[i];
        if (curr.time_from_start != prev.time_from_start) {
            ROS_INFO("Move index %zu into %zu", i, end_idx);
            if (end_idx != i) {
                traj.points[end_idx] = std::move(curr);
            }
            end_idx++;
        }
    }
    traj.points.resize(end_idx);
}

bool PlannerInterface::isPathValid(const std::vector<RobotState>& path) const
{
    for (size_t i = 1; i < path.size(); ++i) {
        if (!m_checker->isStateToStateValid(path[i - 1], path[i])) {
            ROS_ERROR_STREAM("path between " << path[i - 1] << " and " << path[i] << " is invalid (" << i - 1 << " -> " << i << ")");
            return false;
        }
    }
    return true;
}

void PlannerInterface::postProcessPath(std::vector<RobotState>& path) const
{
    const bool check_planned_path = true;
    if (check_planned_path && !isPathValid(path)) {
        ROS_ERROR("Planned path is invalid");
    }

    // shortcut path
    if (m_params.shortcut_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate planned path with %zu waypoints before shortcutting.", path.size());
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);
        } else {
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);
        }
    }

    // interpolate path
    if (m_params.interpolate_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate trajectory");
        }
    }
}

void PlannerInterface::convertJointVariablePathToJointTrajectory(
    const std::vector<RobotState>& path,
    trajectory_msgs::JointTrajectory& traj) const
{
    traj.header.frame_id = m_params.planning_frame;
    traj.joint_names = m_robot->getPlanningJoints();
    traj.points.clear();
    traj.points.reserve(path.size());
    for (const auto& point : path) {
        trajectory_msgs::JointTrajectoryPoint traj_pt;
        traj_pt.positions = point;
        traj.points.push_back(std::move(traj_pt));
    }
}

bool PlannerInterface::writePath(
    const moveit_msgs::RobotState& ref,
    const moveit_msgs::RobotTrajectory& traj) const
{
    boost::filesystem::path p(m_params.plan_output_dir);

    try {
        if (!boost::filesystem::exists(p)) {
            ROS_INFO("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            ROS_ERROR("Failed to log path. %s is not a directory", m_params.plan_output_dir.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        ROS_ERROR("Failed to create plan output directory %s", p.native().c_str());
        return false;
    }

    std::stringstream ss_filename;
    auto now = clock::now();
    ss_filename << "path_" << now.time_since_epoch().count();
    p /= ss_filename.str();

    std::ofstream ofs(p.native());
    if (!ofs.is_open()) {
        return false;
    }

    ROS_INFO("Log path to %s", p.native().c_str());

    // write header
    for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
        const std::string& var_name = m_robot->getPlanningJoints()[vidx];
        ofs << var_name; // TODO: sanitize variable name for csv?
        if (vidx != m_robot->jointVariableCount() - 1) {
            ofs << ',';
        }
    }
    ofs << '\n';

    const size_t wp_count = std::max(
            traj.joint_trajectory.points.size(),
            traj.multi_dof_joint_trajectory.points.size());
    for (size_t widx = 0; widx < wp_count; ++widx) {
        // fill the complete robot state
        moveit_msgs::RobotState state = ref;

        if (widx < traj.joint_trajectory.points.size()) {
            const trajectory_msgs::JointTrajectoryPoint& wp =
                    traj.joint_trajectory.points[widx];
            const size_t joint_count = traj.joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string& joint_name =
                        traj.joint_trajectory.joint_names[jidx];
                double vp = wp.positions[jidx];
                auto it = std::find(
                        state.joint_state.name.begin(),
                        state.joint_state.name.end(),
                        joint_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx = std::distance(state.joint_state.name.begin(), it);
                    state.joint_state.position[tvidx] = vp;
                }
            }
        }
        if (widx < traj.multi_dof_joint_trajectory.points.size()) {
            const trajectory_msgs::MultiDOFJointTrajectoryPoint& wp =
                    traj.multi_dof_joint_trajectory.points[widx];
            const size_t joint_count = traj.multi_dof_joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string& joint_name =
                        traj.multi_dof_joint_trajectory.joint_names[jidx];
                const geometry_msgs::Transform& t = wp.transforms[jidx];
                auto it = std::find(
                        state.multi_dof_joint_state.joint_names.begin(),
                        state.multi_dof_joint_state.joint_names.end(),
                        joint_name);
                if (it != state.multi_dof_joint_state.joint_names.end()) {
                    size_t tvidx = std::distance(state.multi_dof_joint_state.joint_names.begin(), it);
                    state.multi_dof_joint_state.transforms[tvidx] = t;
                }
            }
        }

        // write the planning variables out to file
        for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
            const std::string& var_name = m_robot->getPlanningJoints()[vidx];
            const bool var_is_mdof = false; // TODO: multi-dof joints in robot model
            if (var_is_mdof) {

            } else {
                auto it = std::find(
                        state.joint_state.name.begin(),
                        state.joint_state.name.end(),
                        var_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx = std::distance(state.joint_state.name.begin(), it);
                    double vp = state.joint_state.position[tvidx];
                    ofs << vp;
                    if (vidx != m_robot->jointVariableCount() - 1) {
                        ofs << ',';
                    }
                }
            }
        }
        ofs << '\n';
    }

    return true;
}

} // namespace motion
} // namespace sbpl
