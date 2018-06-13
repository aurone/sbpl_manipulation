////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018, Fahad Islam
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

/// \author Fahad Islam

// standard includes
#include <stdio.h>
#include <stdlib.h>

// system includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <tf_conversions/tf_eigen.h>

// project includes
#include <smpl/ros/zero_time_planner.h>

namespace sbpl {
namespace motion {

const char* PI_LOGGER_ZERO = "simple_zero";

ZeroTimePlanner::ZeroTimePlanner(
	const RobotState& start_state,
	const GoalConstraint& goal,
	ManipLattice* manip_space,
	WorkspaceLatticeZero* task_space,
	ARAStar* planner,
	ARAStarZero* planner_zero)
:
    m_goal(goal),
    m_start_state(start_state),
    m_manip_space(manip_space),
    m_task_space(task_space),
    m_planner(planner),
    m_planner_zero(planner_zero)
{
}

void ZeroTimePlanner::PreProcess()
{
	while (true) {

        // 1. SAMPLE ATTRACTOR
        int maximum_tries = 10000;
        RobotState attractor;
        // also sets the attractor as goal for heuristic functions
        int attractor_state_id = m_task_space->SampleAttractorState(m_regions, attractor, maximum_tries);
        if (attractor_state_id == -1) {
            ROS_INFO("Converged! I tried for %d samples", maximum_tries);
            break;
        }

        std::vector<RobotState> path;
#if 0
        // 2. PLAN PATH TO ACTUAL GOAL
        m_manip_space->setStart(attractor);
        const int start_id_manip_space = m_manip_space->getStartStateID();
        if (start_id_manip_space == -1) {
            ROS_ERROR("No start state has been set in manip lattice");
            return;
        }

        if (m_planner->set_start(start_id_manip_space) == 0) {
            ROS_ERROR("Failed to set start state");
            return;
        }

        m_manip_space->setGoal(m_goal);

        // set sbpl planner goal
        const int goal_id = m_manip_space->getGoalStateID();
        if (goal_id == -1) {
            ROS_ERROR("No goal state has been set");
            return;
        }

        if (m_planner->set_goal(goal_id) == 0) {
            ROS_ERROR("Failed to set planner goal state");
            return;
        }

        bool b_ret = false;
        std::vector<int> solution_state_ids;

        // reinitialize the search space
        m_planner->force_planning_from_scratch();

        // plan
        int m_sol_cost;
        b_ret = m_planner->replan(m_req.allowed_planning_time, &solution_state_ids, &m_sol_cost);

        // check if an empty plan was received.
        if (b_ret && solution_state_ids.size() <= 0) {
            ROS_WARN_NAMED(PI_LOGGER_ZERO, "Path returned by the planner is empty?");
            b_ret = false;
        }

        if (!b_ret) {
            ROS_WARN("Solution not found");
            continue;
        }

        // if a path is returned, then pack it into msg form
        if (b_ret && (solution_state_ids.size() > 0)) {
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "Planning succeeded");
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Final): %d", m_planner->get_n_expands());
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Path Length (states): %zu", solution_state_ids.size());
            ROS_INFO_NAMED(PI_LOGGER_ZERO, "  Solution Cost: %d", m_sol_cost);

            if (!m_manip_space->extractPath(solution_state_ids, path)) {
                ROS_ERROR("Failed to convert state id path to joint variable path");
                return;
            }
        }
        // free manip space
        m_manip_space->ClearStates();
        m_planner->force_planning_from_scratch_and_free_memory();
#endif
        // 3. COMPUTE REACHABILITY
        m_task_space->UpdateSearchMode(REACHABILITY);

        // reinitialize the search space
        m_planner_zero->force_planning_from_scratch();

        // reachability search
        int radius = m_planner_zero->compute_reachability(1000000, attractor_state_id);

        // free task space
        m_task_space->ClearStates();
        m_planner_zero->force_planning_from_scratch_and_free_memory();

        // 4. ADD REGION
        region r;
        r.radius = radius;
        r.state = attractor;
        r.path = path;
        m_regions.push_back(r);

        ROS_INFO("Radius %d, Regions so far %zu", radius, m_regions.size());
    }

    m_task_space->PruneRegions(m_regions);
    WriteRegions();
}

void ZeroTimePlanner::Query(std::vector<RobotState>& path)
{
	if (m_regions.empty()) {	
        ReadRegions();
	}
    m_task_space->UpdateSearchMode(QUERY);

    RobotState start_state;
#if 1
    start_state = m_start_state;
#else	// select random start
    while (!m_task_space->SampleRobotState(start_state));
#endif

    if (!m_task_space->IsRobotStateInStartRegion(start_state)) {
    	ROS_ERROR("Query state outside start region");
    	return;
    }

    int reg_idx = m_task_space->FindRegionContainingState(start_state, m_regions);

    if (reg_idx == -1) {
        ROS_ERROR("Query start state not covered");
        return;
    }

    // set start
    m_task_space->setStart(start_state);
    const int start_id = m_task_space->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set in manip lattice");
        return;
    }

    if (m_planner_zero->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state");
        return;
    }

    GoalConstraint goal;
    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = m_regions[reg_idx].state;
	m_task_space->setGoal(goal);

    // set sbpl planner goal
    const int goal_id = m_task_space->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return;
    }

    if (m_planner_zero->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return;
    }

    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner_zero->force_planning_from_scratch();

    // plan
    int m_sol_cost;
    b_ret = m_planner_zero->replan(10, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER_ZERO, "Path returned by the planner is empty?");
        b_ret = false;
    }

    if (!b_ret) {
        ROS_ERROR("Planner failed in query phase");
    }

    // if a path is returned, then pack it into msg form
    std::vector<RobotState> path_to_attractor;
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "Planning succeeded");
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Initial): %d", m_planner_zero->get_n_expands_init_solution());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Num Expansions (Final): %d", m_planner_zero->get_n_expands());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Epsilon (Initial): %0.3f", m_planner_zero->get_initial_eps());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Epsilon (Final): %0.3f", m_planner_zero->get_solution_eps());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Time (Initial): %0.3f", m_planner_zero->get_initial_eps_planning_time());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Time (Final): %0.3f", m_planner_zero->get_final_eps_planning_time());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_DEBUG_NAMED(PI_LOGGER_ZERO, "  Solution Cost: %d", m_sol_cost);

        if (!m_task_space->extractPath(solution_state_ids, path_to_attractor)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return;
        }

        path = path_to_attractor;
        path.insert(
            path.end(),
            m_regions[reg_idx].path.begin(),
            m_regions[reg_idx].path.end());
    }

    m_task_space->ClearStates();
    m_planner_zero->force_planning_from_scratch_and_free_memory();
}

void ZeroTimePlanner::WriteRegions()
{
    // sort
    std::sort(m_regions.begin(), m_regions.end(), [] (const region &a,
              const region &b)
    {
        return (a.radius > b.radius);
    });

	ROS_INFO("Writing regions to file");
    boost::filesystem::path myFile = boost::filesystem::current_path() / "myfile.dat";
    boost::filesystem::ofstream ofs(myFile);
    boost::archive::text_oarchive ta(ofs);
    ta << m_regions;
}

void ZeroTimePlanner::ReadRegions()
{
	ROS_INFO("Reading regions from file");
	boost::filesystem::path myFile = boost::filesystem::current_path() / "myfile.dat";
    boost::filesystem::ifstream ifs(myFile/*.native()*/);
    boost::archive::text_iarchive ta(ifs);
    ta >> m_regions;
}

ZeroTimePlanner::~ZeroTimePlanner()
{
}

} // namespace motion
} // namespace sbpl
