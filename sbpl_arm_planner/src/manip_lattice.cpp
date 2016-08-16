////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <sbpl_arm_planner/manip_lattice.h>

// standard includes
#include <sstream>

// system includes
#include <Eigen/Dense>
#include <leatherman/viz.h>
#include <leatherman/print.h>
#include <sbpl_geometry_utils/utils.h>

#include <sbpl_arm_planner/manip_heuristic.h>
#include "profiling.h"

auto std::hash<sbpl::manip::ManipLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace sbpl {
namespace manip {

ManipLattice::ManipLattice(
    OccupancyGrid* grid,
    RobotModel* rmodel,
    CollisionChecker* cc,
    ActionSet* as,
    PlanningParams* params)
:
    RobotStateLattice(),
    m_grid(grid),
    m_robot(rmodel),
    m_cc(cc),
    m_as(as),
    m_fk_iface(nullptr),
    m_params(params),
    m_heur(nullptr),
    m_min_limits(),
    m_max_limits(),
    m_continuous(),
    m_near_goal(false),
    m_t_start(),
    m_goal(),
    m_goal_entry(nullptr),
    m_start_entry(nullptr),
    m_HashTableSize(32 * 1024),
    m_Coord2StateIDHashTable(nullptr),
    m_states(),
    m_expanded_states(),
    nh_(),
    m_vpub(),
    m_initialized(false)
{
    m_fk_iface = m_robot->getExtension<ForwardKinematicsInterface>();

    m_vpub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1);

    m_min_limits.resize(m_params->num_joints_);
    m_max_limits.resize(m_params->num_joints_);
    m_continuous.resize(m_params->num_joints_);
    for (int jidx = 0; jidx < m_params->num_joints_; ++jidx) {
        m_min_limits[jidx] = rmodel->minPosLimit(jidx);
        m_max_limits[jidx] = rmodel->maxPosLimit(jidx);
        m_continuous[jidx] = !rmodel->hasPosLimit(jidx);
    }
}

ManipLattice::~ManipLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();

    if (m_Coord2StateIDHashTable) {
        delete [] m_Coord2StateIDHashTable;
        m_Coord2StateIDHashTable = nullptr;
    }
}

int ManipLattice::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    assert(FromStateID >= 0 && FromStateID < (int)m_states.size());
    assert(ToStateID >= 0 && ToStateID < (int)m_states.size());
    return m_heur->GetFromToHeuristic(FromStateID, ToStateID);
}

int ManipLattice::GetGoalHeuristic(int state_id)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    ManipLatticeState* state = m_states[state_id];
    state->heur = m_heur->GetGoalHeuristic(state_id);
    return state->heur;
}

int ManipLattice::GetStartHeuristic(int state_id)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    ManipLatticeState* state = m_states[state_id];
    state->heur = m_heur->GetStartHeuristic(state_id);
    return state->heur;
}

void ManipLattice::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    assert(stateID >= 0 && stateID < (int)m_states.size());

    if (!fOut) {
        fOut = stdout;
    }

    ManipLatticeState* HashEntry = m_states[stateID];

    printJointArray(fOut, HashEntry, bVerbose);
}

void ManipLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    succs->clear();
    costs->clear();

    ROS_DEBUG_NAMED(m_params->expands_log_, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_entry->stateID) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= m_params->num_joints_);

    // log expanded state details
    ROS_DEBUG_NAMED(m_params->expands_log_, "  coord: %s", to_string(parent_entry->coord).c_str());
    ROS_DEBUG_NAMED(m_params->expands_log_, "  angles: %s", to_string(parent_entry->state).c_str());
    ROS_DEBUG_NAMED(m_params->expands_log_, "  ee: (%3d, %3d, %3d)", parent_entry->xyz[0], parent_entry->xyz[1], parent_entry->xyz[2]);
    ROS_DEBUG_NAMED(m_params->expands_log_, "  heur: %d", GetGoalHeuristic(state_id));
    ROS_DEBUG_NAMED(m_params->expands_log_, "  gdiff: (%3d, %3d, %3d)", abs(m_goal.xyz[0] - parent_entry->xyz[0]), abs(m_goal.xyz[1] - parent_entry->xyz[1]), abs(m_goal.xyz[2] - parent_entry->xyz[2]));
//    ROS_DEBUG_NAMED(m_params->expands_log_, "  goal dist: %0.3f", m_grid->getResolution() * bfs_->getDistance(parent_entry->xyz[0], parent_entry->xyz[1], parent_entry->xyz[2]));

    visualizeState(parent_entry->state, "expansion");

    int goal_succ_count = 0;

    std::vector<Action> actions;
    if (!m_as->getActionSet(parent_entry->state, actions)) {
        ROS_WARN("Failed to get actions");
        return;
    }

    ROS_DEBUG_NAMED(m_params->expands_log_, "  actions: %zu", actions.size());

    // check actions for validity
    std::vector<int> succ_coord(m_params->num_joints_, 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(m_params->expands_log_, "    action %zu:", i);
        ROS_DEBUG_NAMED(m_params->expands_log_, "      waypoints: %zu", action.size());

        double dist;
        if (!checkAction(parent_entry->state, action, dist)) {
            continue;
        }

        // compute destination coords
        anglesToCoord(action.back(), succ_coord);

        // get the successor

        // get pose of planning link
        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            continue;
        }

        // discretize planning link pose
        int endeff[3];
        m_grid->worldToGrid(
                tgt_off_pose[0], tgt_off_pose[1], tgt_off_pose[2],
                endeff[0], endeff[1], endeff[2]);

        // check if hash entry already exists, if not then create one
        ManipLatticeState* succ_entry;
        if (!(succ_entry = getHashEntry(succ_coord))) {
            succ_entry = createHashEntry(succ_coord, endeff);
            succ_entry->state = action.back();
            succ_entry->dist = dist;
        }

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(action.back(), tgt_off_pose);
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_entry->stateID);
        }
        else {
            succs->push_back(succ_entry->stateID);
        }
        costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));

        // log successor details
        ROS_DEBUG_NAMED(m_params->expands_log_, "      succ: %zu", i);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        id: %5i", succ_entry->stateID);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        coord: %s", to_string(succ_coord).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        state: %s", to_string(succ_entry->state).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        ee: (%3d, %3d, %3d)", endeff[0], endeff[1], endeff[2]);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        pose: %s", to_string(tgt_off_pose).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        gdiff: (%3d, %3d, %3d)", abs(m_goal.xyz[0] - endeff[0]), abs(m_goal.xyz[1] - endeff[1]), abs(m_goal.xyz[2] - endeff[2]));
        ROS_DEBUG_NAMED(m_params->expands_log_, "        heur: %2d", GetGoalHeuristic(succ_entry->stateID));
        ROS_DEBUG_NAMED(m_params->expands_log_, "        dist: %2d", (int)succ_entry->dist);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
    }

    if (goal_succ_count > 0) {
        ROS_DEBUG_NAMED(m_params->expands_log_, "Got %d goal successors!", goal_succ_count);
    }

    m_expanded_states.push_back(state_id);
}

Stopwatch GetLazySuccsStopwatch("GetLazySuccs", 10);

void ManipLattice::GetLazySuccs(
    int SourceStateID,
    std::vector<int>* SuccIDV,
    std::vector<int>* CostV,
    std::vector<bool>* isTrueCost)
{
    GetLazySuccsStopwatch.start();
    PROFAUTOSTOP(GetLazySuccsStopwatch);

    assert(SourceStateID >= 0 && SourceStateID < m_states.size());

    SuccIDV->clear();
    CostV->clear();
    isTrueCost->clear();

    ROS_DEBUG_NAMED(m_params->expands_log_, "expand state %d", SourceStateID);

    // goal state should be absorbing
    if (SourceStateID == m_goal_entry->stateID) {
        return;
    }

    ManipLatticeState* state_entry = m_states[SourceStateID];

    assert(state_entry);
    assert(state_entry->coord.size() >= m_params->num_joints_);

    // log expanded state details
    ROS_DEBUG_NAMED(m_params->expands_log_, "  coord: %s", to_string(state_entry->coord).c_str());
    ROS_DEBUG_NAMED(m_params->expands_log_, "  angles: %s", to_string(state_entry->state).c_str());
    ROS_DEBUG_NAMED(m_params->expands_log_, "  ee: (%3d, %3d, %3d)", state_entry->xyz[0], state_entry->xyz[1], state_entry->xyz[2]);
    ROS_DEBUG_NAMED(m_params->expands_log_, "  heur: %d", GetGoalHeuristic(SourceStateID));
    ROS_DEBUG_NAMED(m_params->expands_log_, "  gdiff: (%3d, %3d, %3d)", abs(m_goal.xyz[0] - state_entry->xyz[0]), abs(m_goal.xyz[1] - state_entry->xyz[1]), abs(m_goal.xyz[2] - state_entry->xyz[2]));
//    ROS_DEBUG_NAMED(m_params->expands_log_, "  goal dist: %0.3f", m_grid->getResolution() * bfs_->getDistance(state_entry->xyz[0], state_entry->xyz[1], state_entry->xyz[2]));

    const std::vector<double>& source_angles = state_entry->state;
    visualizeState(source_angles, "expansion");

    std::vector<Action> actions;
    if (!m_as->getActionSet(source_angles, actions)) {
        ROS_WARN("Failed to get successors");
        return;
    }

    ROS_DEBUG_NAMED(m_params->expands_log_, "  actions: %zu", actions.size());

    int goal_succ_count = 0;
    std::vector<int> succ_coord(m_params->num_joints_);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(m_params->expands_log_, "    action %zu:", i);
        ROS_DEBUG_NAMED(m_params->expands_log_, "      waypoints: %zu", action.size());

        anglesToCoord(action.back(), succ_coord);

        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            continue;
        }

        int endeff[3];
        m_grid->worldToGrid(tgt_off_pose[0], tgt_off_pose[1], tgt_off_pose[2], endeff[0], endeff[1], endeff[2]);

        const bool succ_is_goal_state = isGoal(action.back(), tgt_off_pose);
        if (succ_is_goal_state) {
            ++goal_succ_count;
        }

        // check if hash entry already exists, if not then create one
        ManipLatticeState* succ_entry;
        if (!(succ_entry = getHashEntry(succ_coord))) {
            succ_entry = createHashEntry(succ_coord, endeff);
            succ_entry->state = action.back();
            succ_entry->dist = 0.0; //dist;
        }

        if (succ_is_goal_state) {
            SuccIDV->push_back(m_goal_entry->stateID);
        }
        else {
            SuccIDV->push_back(succ_entry->stateID);
        }
        CostV->push_back(cost(state_entry, succ_entry, succ_is_goal_state));
        isTrueCost->push_back(false);

        // log successor details
        ROS_DEBUG_NAMED(m_params->expands_log_, "      succ: %zu", i);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        id: %5i", succ_entry->stateID);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        coord: %s", to_string(succ_coord).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        state: %s", to_string(succ_entry->state).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        ee: (%3d, %3d, %3d)", endeff[0], endeff[1], endeff[2]);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        pose: %s", to_string(tgt_off_pose).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        gdiff: (%3d, %3d, %3d)", abs(m_goal.xyz[0] - endeff[0]), abs(m_goal.xyz[1] - endeff[1]), abs(m_goal.xyz[2] - endeff[2]));
        ROS_DEBUG_NAMED(m_params->expands_log_, "        heur: %2d", GetGoalHeuristic(succ_entry->stateID));
        ROS_DEBUG_NAMED(m_params->expands_log_, "        dist: %2d", (int)succ_entry->dist);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        cost: %5d", cost(state_entry, succ_entry, succ_is_goal_state));
    }

    if (goal_succ_count > 0) {
        ROS_DEBUG_NAMED(m_params->expands_log_, "Got %d goal successors!", goal_succ_count);
    }

    m_expanded_states.push_back(SourceStateID);
}

Stopwatch GetTrueCostStopwatch("GetTrueCost", 10);

int ManipLattice::GetTrueCost(int parentID, int childID)
{
    GetTrueCostStopwatch.start();
    PROFAUTOSTOP(GetTrueCostStopwatch);

    ROS_DEBUG_NAMED(m_params->expands_log_, "evaluating cost of transition %d -> %d", parentID, childID);

    assert(parentID >= 0 && parentID < (int)m_states.size());
    assert(childID >= 0 && childID < (int)m_states.size());

    ManipLatticeState* parent_entry = m_states[parentID];
    ManipLatticeState* child_entry = m_states[childID];
    assert(parent_entry && parent_entry->coord.size() >= m_params->num_joints_);
    assert(child_entry && child_entry->coord.size() >= m_params->num_joints_);

    const std::vector<double>& parent_angles = parent_entry->state;
    visualizeState(parent_angles, "expansion");

    std::vector<Action> actions;
    if (!m_as->getActionSet(parent_angles, actions)) {
        ROS_WARN("Failed to get actions");
        return -1;
    }

    const bool goal_edge = (child_entry == m_goal_entry);

    size_t num_actions = 0;

    // check actions for validity and find the valid action with the least cost
    std::vector<int> succ_coord(m_params->num_joints_);
    int best_cost = std::numeric_limits<int>::max();
    for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
        const Action& action = actions[aidx];

        anglesToCoord(action.back(), succ_coord);

        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            continue;
        }

        // check whether this action leads to the child state
        if (goal_edge) {
            // skip actions which don't end up at a goal state
            if (!isGoal(action.back(), tgt_off_pose)) {
                continue;
            }
        }
        else {
            // skip actions which don't end up at the child state
            if (succ_coord != child_entry->coord) {
                continue;
            }
        }

        ROS_DEBUG_NAMED(m_params->expands_log_, "    action %zu:", num_actions++);
        ROS_DEBUG_NAMED(m_params->expands_log_, "      waypoints %zu:", action.size());

        double dist;
        if (!checkAction(parent_angles, action, dist)) {
            continue;
        }

        // get the unique state
        ManipLatticeState* succ_entry = goal_edge ?
                getHashEntry(succ_coord) : child_entry;
        assert(succ_entry);

        const bool is_goal = isGoal(action.back(), tgt_off_pose);
        const int edge_cost = cost(parent_entry, succ_entry, is_goal);
        if (edge_cost < best_cost) {
            best_cost = edge_cost;
        }
    }

    if (best_cost != std::numeric_limits<int>::max()) {
        return best_cost;
    }
    else {
        return -1;
    }
}

void ManipLattice::GetPreds(
    int TargetStateID,
    std::vector<int>* PredIDV,
    std::vector<int>* CostV)
{
    ROS_WARN("GetPreds unimplemented");
}

void ManipLattice::printHashTableHist()
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int  j = 0; j < m_HashTableSize; j++) {
        if ((int)m_Coord2StateIDHashTable[j].size() == 0) {
            s0++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 50) {
            s1++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 100) {
            s50++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 200) {
            s100++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 300) {
            s200++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 400) {
            s300++;
        }
        else {
            slarge++;
        }
    }
    ROS_DEBUG_NAMED(m_params->graph_log_, "hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d", s0,s1, s50, s100, s200,s300,slarge);
}

ManipLatticeState* ManipLattice::getHashEntry(
    const std::vector<int>& coord)
{
    int binid = getHashBin(coord);

    // iterate over the states in the bin and select the perfect match
    for (ManipLatticeState* entry : m_Coord2StateIDHashTable[binid]) {
        if (entry->coord == coord) {
            return entry;
        }
    }
    return nullptr;
}

/// NOTE: const although RobotModel::computePlanningLinkFK used underneath may
/// not be
bool ManipLattice::computePlanningFrameFK(
    const std::vector<double>& state,
    std::vector<double>& pose) const
{
    assert(state.size() == m_params->num_joints_);

    if (!m_fk_iface || !m_fk_iface->computePlanningLinkFK(state, pose)) {
        return false;
    }

    // pose represents T_planning_eef
    Eigen::Affine3d T_planning_tipoff =  // T_planning_eef * T_eef_tipoff
            Eigen::Translation3d(pose[0], pose[1], pose[2]) *
            Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
            Eigen::Translation3d(
                    m_goal.xyz_offset[0],
                    m_goal.xyz_offset[1],
                    m_goal.xyz_offset[2]);
    const Eigen::Vector3d voff(T_planning_tipoff.translation());
    pose[0] = voff.x();
    pose[1] = voff.y();
    pose[2] = voff.z();

    assert(pose.size() == 6);
    return true;
}

ManipLatticeState* ManipLattice::createHashEntry(
    const std::vector<int>& coord,
    int endeff[3])
{
    int i;
    ManipLatticeState* HashEntry = new ManipLatticeState;

    HashEntry->coord = coord;

    memcpy(HashEntry->xyz, endeff, 3 * sizeof(int));

    // assign a stateID to HashEntry to be used
    HashEntry->stateID = m_states.size();
    // insert into the tables
    m_states.push_back(HashEntry);

    // get the hash table bin
    i = getHashBin(HashEntry->coord);

    // insert the entry into the bin
    m_Coord2StateIDHashTable[i].push_back(HashEntry);

    // insert into and initialize the mappings
    int* entry = new int [NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size()-1) {
        ROS_ERROR_NAMED(m_params->graph_log_, "last state has incorrect stateID");
        throw new SBPL_Exception();
    }
    return HashEntry;
}

int ManipLattice::cost(
    ManipLatticeState* HashEntry1,
    ManipLatticeState* HashEntry2,
    bool bState2IsGoal)
{
    return m_params->cost_multiplier_;
}

bool ManipLattice::initEnvironment(ManipHeuristic* heur)
{
    if (!heur) {
        ROS_ERROR_NAMED(m_params->graph_log_, "Heuristic is null");
        return false;
    }

    m_heur = heur;

    ROS_DEBUG_NAMED(m_params->graph_log_, "Initializing environment");

    // initialize environment data
    m_Coord2StateIDHashTable = new std::vector<ManipLatticeState*>[m_HashTableSize];
    m_states.clear();

    // create empty start & goal states

    // TODO: need to be really careful about this...the search should "probably"
    // never generate a unique state that has the same id as the reserved goal
    // state and thus uses the same storage. This case should be rare at the
    // moment since it requires all non-continuous joints to be at their minimum
    // values and all continuous joints to be at their zero positions, but that
    // case will likely produce a bug in the current state
    int endeff[3] = { 0 };
    std::vector<int> coord(m_params->num_joints_, 0);
    m_start_entry = nullptr;
    m_goal_entry = createHashEntry(coord, endeff);
    ROS_DEBUG_NAMED(m_params->graph_log_, "  goal state has state ID %d", m_goal_entry->stateID);

    // compute the cost per cell to be used by heuristic
    computeCostPerCell();

    // set 'environment is initialized' flag
    m_initialized = true;
    ROS_DEBUG_NAMED(m_params->graph_log_, "Environment has been initialized.");
    return true;
}

void ManipLattice::insertStartObserver(ManipLatticeStartObserver* observer)
{
    auto it = std::find(
            m_start_observers.begin(), m_start_observers.end(), observer);
    if (it == m_start_observers.end()) {
        m_start_observers.push_back(observer);
    }
}

void ManipLattice::removeStartObserver(ManipLatticeStartObserver* observer)
{
    auto it = std::find(
            m_start_observers.begin(), m_start_observers.end(), observer);
    if (it != m_start_observers.end()) {
        m_start_observers.erase(it);
    }
}

void ManipLattice::insertGoalObserver(ManipLatticeGoalObserver* observer)
{
    auto it = std::find(
            m_goal_observers.begin(), m_goal_observers.end(), observer);
    if (it == m_goal_observers.end()) {
        m_goal_observers.push_back(observer);
    }
}

void ManipLattice::removeGoalObserver(ManipLatticeGoalObserver* observer)
{
    auto it = std::find(
            m_goal_observers.begin(), m_goal_observers.end(), observer);
    if (it != m_goal_observers.end()) {
        m_goal_observers.erase(it);
    }
}

bool ManipLattice::isGoal(
    const std::vector<double>& angles,
    const std::vector<double>& pose)
{
    switch (m_goal.type) {
    case GoalType::JOINT_STATE_GOAL:
    {
        for (int i = 0; i < m_goal.angles.size(); i++) {
            if (fabs(angles[i] - m_goal.angles[i]) > m_goal.angle_tolerances[i]) {
                return false;
            }
        }
        return true;
    }   break;
    case GoalType::XYZ_RPY_GOAL:
    {
        if (fabs(pose[0] - m_goal.tgt_off_pose[0]) <= m_goal.xyz_tolerance[0] &&
            fabs(pose[1] - m_goal.tgt_off_pose[1]) <= m_goal.xyz_tolerance[1] &&
            fabs(pose[2] - m_goal.tgt_off_pose[2]) <= m_goal.xyz_tolerance[2])
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                double time_to_goal_region = (clock() - m_t_start) / (double)CLOCKS_PER_SEC;
                m_near_goal = true;
                ROS_INFO_NAMED(m_params->expands_log_, "Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %0.4f sec. (after %zu expansions)",
                        pose[0], pose[1], pose[2],
                        m_goal.xyz_tolerance[0],
                        m_goal.tgt_off_pose[0], m_goal.tgt_off_pose[1], m_goal.tgt_off_pose[2],
                        time_to_goal_region,
                        m_expanded_states.size());
            }
            const double droll = angles::ShortestAngleDist(pose[3], m_goal.tgt_off_pose[3]);
            const double dpitch = angles::ShortestAngleDist(pose[4], m_goal.tgt_off_pose[4]);
            const double dyaw = angles::ShortestAngleDist(pose[5], m_goal.tgt_off_pose[5]);
            ROS_DEBUG_NAMED(m_params->expands_log_, "Near goal! (%0.3f, %0.3f, %0.3f)", droll, dpitch, dyaw);
            if (droll < m_goal.rpy_tolerance[0] &&
                dpitch < m_goal.rpy_tolerance[1] &&
                dyaw < m_goal.rpy_tolerance[2])
            {
                return true;
            }
        }
    }   break;
    case GoalType::XYZ_GOAL:
    {
        if (fabs(pose[0] - m_goal.tgt_off_pose[0]) <= m_goal.xyz_tolerance[0] &&
            fabs(pose[1] - m_goal.tgt_off_pose[1]) <= m_goal.xyz_tolerance[1] &&
            fabs(pose[2] - m_goal.tgt_off_pose[2]) <= m_goal.xyz_tolerance[2])
        {
            return true;
        }
    }   break;
    default:
    {
        ROS_ERROR_NAMED(m_params->graph_log_, "Unknown goal type.");
    }   break;
    }

    return false;
}

int ManipLattice::getActionCost(
    const std::vector<double>& from_config,
    const std::vector<double>& to_config,
    int dist)
{
    int num_prims = 0, cost = 0;
    double diff = 0, max_diff = 0;

    if (from_config.size() != to_config.size()) {
        return -1;
    }

    /* NOTE: Not including forearm roll OR wrist roll movement to calculate mprim cost */

    for (size_t i = 0; i < 6; i++) {
        if (i == 4) {
            continue;
        }

        diff = angles::ShortestAngleDist(from_config[i], to_config[i]);
        if (max_diff < diff) {
            max_diff = diff;
        }
    }

    num_prims = max_diff / m_params->max_mprim_offset_ + 0.5;
    cost = num_prims * m_params->cost_multiplier_;

    std::vector<double> from_config_norm(from_config.size());
    for (size_t i = 0; i < from_config.size(); ++i) {
        from_config_norm[i] = angles::NormalizeAngle(from_config[i]);
    }

    return cost;
}

bool ManipLattice::checkAction(
    const RobotState& state,
    const Action& action,
    double& dist) const
{
    std::uint32_t violation_mask = 0x00000000;
    int plen = 0;
    int nchecks = 0;
    dist = 0.0;

    // check intermediate states for collisions
    for (size_t iidx = 0; iidx < action.size(); ++iidx) {
        const RobotState& istate = action[iidx];
        ROS_DEBUG_NAMED(m_params->expands_log_, "        %zu: %s", iidx, to_string(istate).c_str());

        // check joint limits
        if (!m_robot->checkJointLimits(istate)) {
            ROS_DEBUG_NAMED(m_params->expands_log_, "        -> violates joint limits");
            violation_mask |= 0x00000001;
            break;
        }

        // TODO/NOTE: this can result in an unnecessary number of collision
        // checks per each action; leaving commented here as it might hint at
        // an optimization where actions are checked at a coarse resolution as
        // a way of speeding up overall collision checking; in that case, the
        // isStateToStateValid function on CollisionChecker would have semantics
        // meaning "collision check a waypoint path without including the
        // endpoints".
//        // check for collisions
//        if (!m_cc->isStateValid(istate, m_params->verbose_collisions_, false, dist))
//        {
//            ROS_DEBUG_NAMED(m_params->expands_log_, "        -> in collision (dist: %0.3f)", dist);
//            violation_mask |= 0x00000002;
//            break;
//        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions along path from parent to first waypoint
    if (!m_cc->isStateToStateValid(state, action[0], plen, nchecks, dist)) {
        ROS_DEBUG_NAMED(m_params->expands_log_, "        -> path to first waypoint in collision (dist: %0.3f, path_length: %d)", dist, plen);
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between waypoints
    for (size_t j = 1; j < action.size(); ++j) {
        const RobotState& prev_istate = action[j - 1];
        const RobotState& curr_istate = action[j];
        if (!m_cc->isStateToStateValid(
                prev_istate, curr_istate, plen, nchecks, dist))
        {
            ROS_DEBUG_NAMED(m_params->expands_log_, "        -> path between waypoints %zu and %zu in collision (dist: %0.3f, path_length: %d)", j - 1, j, dist, plen);
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

bool ManipLattice::setStartState(const RobotState& state)
{
    ROS_DEBUG_NAMED(m_params->graph_log_, "set the start state");

    if ((int)state.size() < m_params->num_joints_) {
        ROS_ERROR_NAMED(m_params->graph_log_, "start state does not contain enough joint positions");
        return false;
    }

    ROS_DEBUG_NAMED(m_params->graph_log_, "  state: %s", to_string(state).c_str());

    // get joint positions of starting configuration
    std::vector<double> pose(6, 0.0);
    if (!computePlanningFrameFK(state, pose)) {
        ROS_WARN(" -> unable to compute forward kinematics");
        return false;
    }
    ROS_DEBUG_NAMED(m_params->graph_log_, "  planning link pose: { x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f }", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    // check joint limits of starting configuration
    if (!m_robot->checkJointLimits(state, true)) {
        ROS_WARN(" -> violates the joint limits");
        return false;
    }

    // check if the start configuration is in collision
    double dist = 0.0;
    if (!m_cc->isStateValid(state, true, false, dist)) {
        ROS_WARN(" -> in collision (distance to nearest obstacle %0.3fm)", dist);
        return false;
    }

    visualizeState(state, "start_config");

    // get arm position in environment
    std::vector<int> start_coord(m_params->num_joints_);
    anglesToCoord(state, start_coord);
    ROS_DEBUG_NAMED(m_params->graph_log_, "  coord: %s", to_string(start_coord).c_str());

    int endeff[3];
    m_grid->worldToGrid(pose[0], pose[1], pose[2], endeff[0], endeff[1], endeff[2]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "  pose: (%d, %d, %d)", endeff[0], endeff[1], endeff[2]);
    // TODO: check for within grid bounds?

    ManipLatticeState* start_entry;
    if (!(start_entry = getHashEntry(start_coord))) {
        start_entry = createHashEntry(start_coord, endeff);
        start_entry->state = state;
        start_entry->dist = dist;
    }

    m_start_entry = start_entry;

    for (auto obs : m_start_observers) {
        obs->updateStart(state);
    }

    return true;
}

/// \brief Set a full joint configuration goal.
bool ManipLattice::setGoalConfiguration(
    const std::vector<double>& goal,
    const std::vector<double>& goal_tolerances)
{
    if (!m_initialized) {
        ROS_ERROR_NAMED(m_params->graph_log_, "Cannot set goal position because environment is not initialized.");
        return false;
    }

    // compute the goal pose
    std::vector<std::vector<double>> goals_6dof;
    std::vector<double> pose;
    if (!computePlanningFrameFK(goal, pose)) {
        SBPL_WARN("Could not compute planning link FK for given goal configuration!");
        return false;
    }
    goals_6dof.push_back(pose);

    std::vector<std::vector<double>> offsets_6dof(1, std::vector<double>(3, 0.0));

    // made up goal tolerance (it should not be used in with 7dof goals anyways)
    std::vector<std::vector<double>> tolerances_6dof(1, std::vector<double>(6, 0.05));

    if (!setGoalPosition(goals_6dof, offsets_6dof, tolerances_6dof)) {
	   ROS_WARN("Failed to set goal position");
	   return false;
    }

    // fill in m_goal
    m_goal.angles = goal;
    m_goal.angle_tolerances = goal_tolerances;
    m_goal.type = GoalType::JOINT_STATE_GOAL;

    for (auto obs : m_goal_observers) {
        obs->updateGoal(m_goal);
    }
    return true;
}

/// \brief Set a 6-dof goal pose for the tip link.
///
/// \param goals A list of goal poses/positions for offsets from the tip link.
///     The format of each element is { x_i, y_i, z_i, R_i, P_i, Y_i, 6dof? }
///     where the first 6 elements specify the goal pose of the end effector and
///     the 7th element is a flag indicating whether orientation constraints are
///     required.
///
/// \param offsets A list of offsets from the tip link corresponding to \p
///     goals. The goal condition and the heuristic values will be computed
///     relative to these offsets.
///
/// \param tolerances A list of goal pose/position tolerances corresponding to
///     the \p goals. The format of each element is { dx_i, dy_i, dz_i, dR_i,
///     dP_i, dY_i } in meters/radians.
bool ManipLattice::setGoalPosition(
    const std::vector<std::vector<double>>& goals,
    const std::vector<std::vector<double>>& offsets,
    const std::vector<std::vector<double>>& tolerances)
{
    if (!m_initialized) {
        ROS_ERROR_NAMED(m_params->graph_log_, "Cannot set goal position because environment is not initialized.");
        return false;
    }

    // check arguments

    if (goals.empty()) {
        ROS_ERROR_NAMED(m_params->graph_log_, "goals vector is empty");
        return false;
    }

    for (const auto& goal : goals) {
        if (goal.size() != 7) {
            ROS_ERROR_NAMED(m_params->graph_log_, "goal element has incorrect format");
            return false;
        }
    }

    if (offsets.size() != goals.size()) {
        ROS_ERROR_NAMED(m_params->graph_log_, "setGoalPosition requires as many offset elements as goal elements");
        return false;
    }

    for (const auto& offset : offsets) {
        if (offset.size() != 3) {
            ROS_ERROR_NAMED(m_params->graph_log_, "offset element has incorrect format");
            return false;
        }
    }

    if (tolerances.size() != goals.size()) {
        ROS_ERROR_NAMED(m_params->graph_log_, "setGoalPosition requires as many tolerance elements as goal elements");
        return false;
    }

    for (const auto& tol : tolerances) {
        if (tol.size() != 6) {
            ROS_ERROR_NAMED(m_params->graph_log_, "tolerance element has incorrect format");
            return false;
        }
    }

    m_goal.pose = goals[0];

    std::copy(offsets[0].begin(), offsets[0].end(), m_goal.xyz_offset);
    std::copy(tolerances[0].begin(), tolerances[0].begin() + 3, m_goal.xyz_tolerance);
    std::copy(tolerances[0].begin() + 3, tolerances[0].begin() + 6, m_goal.rpy_tolerance);

    m_goal.type = (GoalType)((int)goals[0][6]);

    std::vector<double> tgt_off_pose = getTargetOffsetPose(m_goal.pose);
    m_goal.tgt_off_pose = tgt_off_pose;

    auto goal_markers = viz::getPosesMarkerArray(
            { tgt_off_pose },
            m_grid->getReferenceFrame(),
            "target_goal");
    m_vpub.publish(goal_markers);

    // set goal hash entry
    m_grid->worldToGrid(tgt_off_pose.data(), m_goal.xyz);

    for (int i = 0; i < m_params->num_joints_; i++) {
        m_goal_entry->coord[i] = 0;
    }

    ROS_DEBUG_NAMED(m_params->graph_log_, "time: %f", clock() / (double)CLOCKS_PER_SEC);
    ROS_DEBUG_NAMED(m_params->graph_log_, "A new goal has been set.");
    ROS_DEBUG_NAMED(m_params->graph_log_, "    grid (cells): (%d, %d, %d)", m_goal_entry->xyz[0], m_goal_entry->xyz[1], m_goal_entry->xyz[2]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "    xyz (meters): (%0.2f, %0.2f, %0.2f)", m_goal.pose[0], m_goal.pose[1], m_goal.pose[2]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "    tol (meters): %0.3f", m_goal.xyz_tolerance[0]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "    rpy (radians): (%0.2f, %0.2f, %0.2f)", m_goal.pose[3], m_goal.pose[4], m_goal.pose[5]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "    tol (radians): %0.3f", m_goal.rpy_tolerance[0]);

    m_near_goal = false;
    m_t_start = clock();

    for (auto obs : m_goal_observers) {
        obs->updateGoal(m_goal);
    }

    return true;
}

void ManipLattice::printJointArray(
    FILE* fOut,
    ManipLatticeState* HashEntry,
    bool bVerbose)
{
    std::vector<double> angles(m_params->num_joints_, 0.0);

    std::stringstream ss;

    if (HashEntry->stateID == m_goal_entry->stateID) {
        ss << "<goal state>";
    }
    else {
        coordToAngles(HashEntry->coord, angles);
        if (bVerbose) {
            ss << "angles: ";
        }
        ss << "{ ";
        for (size_t i = 0; i < angles.size(); ++i) {
            ss << std::setprecision(3) << angles[i];
            if (i != angles.size() - 1) {
                ss << ", ";
            }
        }
        ss << " }";
    }

    if (fOut == stdout) {
        ROS_DEBUG_NAMED(m_params->graph_log_, "%s", ss.str().c_str());
    }
    else if (fOut == stderr) {
        ROS_WARN("%s", ss.str().c_str());
    }
    else {
        fprintf(fOut, "%s\n", ss.str().c_str());
    }
}

void ManipLattice::getExpandedStates(
    std::vector<std::vector<double>>& states) const
{
    std::vector<double> angles(m_params->num_joints_,0);
    std::vector<double> state(7, 0); // { x, y, z, r, p, y, heur }

    for (size_t i = 0; i < m_expanded_states.size(); ++i) {
        if (!StateID2Angles(m_expanded_states[i], angles)) {
            continue;
        }
        computePlanningFrameFK(angles, state);
        state[6] = m_states[m_expanded_states[i]]->heur;
        states.push_back(state);
        ROS_DEBUG_NAMED(m_params->graph_log_, "[%d] id: %d  xyz: %s", int(i), m_expanded_states[i], to_string(state).c_str());
    }
}

void ManipLattice::computeCostPerCell()
{
    ROS_WARN_NAMED(m_params->graph_log_, "Cell Cost: Uniform 100");
    m_params->cost_per_cell_ = 100;
}

bool ManipLattice::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    std::vector<RobotState> opath;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (idpath.size() == 1) {
        const int state_id = idpath[0];

        if (state_id == getGoalStateID()) {
            RobotState angles;
            if (!StateID2Angles(getStartStateID(), angles)) {
                ROS_ERROR_NAMED(m_params->graph_log_, "Failed to get robot state from state id %d", getStartStateID());
                return false;
            }

            opath.push_back(std::move(angles));
        }
        else {
            RobotState angles;
            if (!StateID2Angles(state_id, angles)) {
                ROS_ERROR_NAMED(m_params->graph_log_, "Failed to get robot state from state id %d", state_id);
                return false;
            }

            opath.push_back(std::move(angles));
        }

        return true;
    }

    if (idpath[0] == getGoalStateID()) {
        ROS_ERROR_NAMED(m_params->graph_log_, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    // grab the first point
    {
        RobotState angles;
        if (!StateID2Angles(idpath[0], angles)) {
            ROS_ERROR_NAMED(m_params->graph_log_, "Failed to get robot state from state id %d", idpath[0]);
            return false;
        }
        opath.push_back(std::move(angles));
    }

    // grab the rest of the points
    for (size_t i = 1; i < idpath.size(); ++i) {
        const int prev_id = idpath[i - 1];
        const int curr_id = idpath[i];

        if (prev_id == getGoalStateID()) {
            ROS_ERROR_NAMED(m_params->graph_log_, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            // find the goal state corresponding to the cheapest valid action

            ManipLatticeState* prev_entry = m_states[prev_id];
            const RobotState& prev_state = prev_entry->state;

            std::vector<Action> actions;
            if (!m_as->getActionSet(prev_state, actions)) {
                ROS_ERROR_NAMED(m_params->graph_log_, "Failed to get actions while extracting the path");
                return false;
            }

            ManipLatticeState* best_goal_state = nullptr;
            std::vector<int> succ_coord(m_params->num_joints_);
            int best_cost = std::numeric_limits<int>::max();
            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                std::vector<double> tgt_off_pose;
                if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
                    ROS_WARN("Failed to compute FK for planning frame");
                    continue;
                }

                // skip non-goal states
                if (!isGoal(action.back(), tgt_off_pose)) {
                    continue;
                }

                // check the validity of this transition
                double dist;
                if (!checkAction(prev_state, action, dist)) {
                    continue;
                }

                anglesToCoord(action.back(), succ_coord);
                ManipLatticeState* succ_entry = getHashEntry(succ_coord);
                assert(succ_entry);

                const int edge_cost = cost(prev_entry, succ_entry, true);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_goal_state = succ_entry;
                }
            }

            if (!best_goal_state) {
                ROS_ERROR_NAMED(m_params->graph_log_, "Failed to find valid goal successor during path extraction");
                return false;
            }

            opath.push_back(best_goal_state->state);
        }
        else {
            RobotState curr_state;
            if (!StateID2Angles(curr_id, curr_state)) {
                ROS_ERROR_NAMED(m_params->graph_log_, "Failed to get robot state from state id %d", curr_id);
                return false;
            }

            opath.push_back(std::move(curr_state));
        }
    }

    // we made it!
    path = std::move(opath);
    return true;
}

/// \brief Get the (heuristic) distance from the planning link pose to the start
double ManipLattice::getStartDistance(double x, double y, double z)
{
    return m_heur->getMetricStartDistance(x, y, z);
}

double ManipLattice::getStartDistance(const std::vector<double>& pose)
{
    std::vector<double> tipoff_pose = getTargetOffsetPose(pose);
    return getStartDistance(tipoff_pose[0], tipoff_pose[1], tipoff_pose[2]);
}

/// \brief Get the (heuristic) distance from the planning frame position to the
///     goal
double ManipLattice::getGoalDistance(double x, double y, double z)
{
    return m_heur->getMetricGoalDistance(x, y, z);
}

// \brief Get the (heuristic) distance from the planning link pose to the goal
double ManipLattice::getGoalDistance(const std::vector<double>& pose)
{
    std::vector<double> tipoff_pose = getTargetOffsetPose(pose);
    return getGoalDistance(tipoff_pose[0], tipoff_pose[1], tipoff_pose[2]);
}

const ManipLatticeState* ManipLattice::getHashEntry(int state_id) const
{
    if (state_id < 0 || state_id >= m_states.size()) {
        return nullptr;
    }

    return m_states[state_id];
}

/// \brief Return the ID of the goal state or -1 if no goal has been set.
int ManipLattice::getGoalStateID() const
{
    return m_goal_entry ? m_goal_entry->stateID : -1;
}

/// \brief Return the ID of the start state or -1 if no start has been set.
///
/// This returns the reserved id corresponding to all states which are goal
/// states and not the state id of any particular unique state.
int ManipLattice::getStartStateID() const
{
    return m_start_entry ? m_start_entry->stateID : -1;
}

/// \brief Return the 6-dof goal pose for the tip link.
///
/// Return the 6-dof goal pose for the tip link, as last set by
/// setGoalPosition(). If no goal has been set, the returned vector is empty.
const std::vector<double>& ManipLattice::getGoal() const
{
    return m_goal.pose;
}

/// \brief Return the 6-dof goal pose for the offset from the tip link.
std::vector<double> ManipLattice::getTargetOffsetPose(
    const std::vector<double>& tip_pose) const
{
    // pose represents T_planning_eef
    Eigen::Affine3d T_planning_tipoff = // T_planning_eef * T_eef_tipoff
            Eigen::Translation3d(tip_pose[0], tip_pose[1], tip_pose[2]) *
            Eigen::AngleAxisd(tip_pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(tip_pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(tip_pose[3], Eigen::Vector3d::UnitX()) *
            Eigen::Translation3d(
                    m_goal.xyz_offset[0],
                    m_goal.xyz_offset[1],
                    m_goal.xyz_offset[2]);
    const Eigen::Vector3d voff(T_planning_tipoff.translation());
    return { voff.x(), voff.y(), voff.z(), tip_pose[3], tip_pose[4], tip_pose[5] };
}

const GoalConstraint& ManipLattice::getGoalConstraints() const
{
    return m_goal;
}

/// \brief Return the full joint configuration goal.
///
/// Return the full joint configuration goal, as last set by
/// setGoalConfiguration().
std::vector<double> ManipLattice::getGoalConfiguration() const
{
    return m_goal.angles;
}

/// \brief Get the (heuristic) distance from the planning frame position to the
///     start
std::vector<double> ManipLattice::getStartConfiguration() const
{
    if (m_start_entry) {
        return m_start_entry->state;
    }
    else {
        return std::vector<double>();
    }
}

void ManipLattice::visualizeState(
    const std::vector<double>& jvals,
    const std::string& ns) const
{
    auto ma = m_cc->getCollisionModelVisualization(jvals);
    for (auto& marker : ma.markers) {
        marker.ns = ns;
    }
    m_vpub.publish(ma);
}

bool ManipLattice::StateID2Angles(
    int stateID,
    std::vector<double>& angles) const
{
    if (stateID < 0 || stateID >= m_states.size()) {
        return false;
    }
    if (stateID == m_goal_entry->stateID) {
        ROS_ERROR("You should stop caring about the values within the goal state");
        return false;
    }

    ManipLatticeState* entry = m_states[stateID];
    assert(entry);

    angles = entry->state;
    return true;
}

inline
unsigned int ManipLattice::intHash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

inline
unsigned int ManipLattice::getHashBin(
    const std::vector<int>& coord)
{
    int val = 0;

    for (size_t i = 0; i < coord.size(); i++) {
        val += intHash(coord[i]) << i;
    }

    return intHash(val) & (m_HashTableSize - 1);
}

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin
// 0, ...
inline
void ManipLattice::coordToAngles(
    const std::vector<int>& coord,
    std::vector<double>& angles) const
{
    angles.resize(coord.size());
    for (size_t i = 0; i < coord.size(); ++i) {
        if (m_continuous[i]) {
            angles[i] = coord[i] * m_params->coord_delta_[i];
        }
        else {
            angles[i] = m_min_limits[i] + coord[i] * m_params->coord_delta_[i];
        }
    }
}

inline
void ManipLattice::anglesToCoord(
    const std::vector<double>& angle,
    std::vector<int>& coord) const
{
    assert((int)angle.size() == m_params->num_joints_ &&
            (int)coord.size() == m_params->num_joints_);

    for (size_t i = 0; i < angle.size(); ++i) {
        if (m_continuous[i]) {
            double pos_angle = angles::NormalizeAnglePositive(angle[i]);

            coord[i] = (int)((pos_angle + m_params->coord_delta_[i] * 0.5) / m_params->coord_delta_[i]);

            if (coord[i] == m_params->coord_vals_[i]) {
                coord[i] = 0;
            }
        }
        else {
            coord[i] = (int)(((angle[i] - m_min_limits[i]) / m_params->coord_delta_[i]) + 0.5);
        }
    }
}

} // namespace manip
} // namespace sbpl
