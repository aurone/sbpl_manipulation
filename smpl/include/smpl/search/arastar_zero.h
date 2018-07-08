////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush, Fahad Islam
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

/// \author Andrew Dornbush, Fahad Islam

#ifndef SMPL_ARASTAR_ZERO_H
#define SMPL_ARASTAR_ZERO_H

// standard includes
#include <assert.h>
#include <algorithm>
#include <set>
#include <unordered_map>

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/workspace_lattice_zero.h>
#include <smpl/intrusive_heap.h>
#include <smpl/time.h>

namespace sbpl {

/// An implementation of the ARA* (Anytime Repairing A*) search algorithm. This
/// algorithm runs a series of weighted A* searches with decreasing bounds on
/// suboptimality to return the best solution found within a given time bound.
/// The search intelligently reuses its search tree between successive
/// iterations for improved efficiency, rather than starting each new
/// weighted-A* iteration from scratch.
///
/// This class maintains the state of the search procedure between calls to
/// replan(), allowing the search to resume from where it left off when the
/// scenario (start, goal, and edge costs in the graph) doesn't change between
/// calls. This can be used to dedicate more time to searching in the event the
/// search fails to find a solution within the given time and to allow solutions
/// to be returned quickly and allowing the search to continue improving the
/// solution given more time. To implement this, several assumptions about the
/// implementation of the graph and heuristic are made:
///
/// * The state IDs are constant between calls to replan(). If the state ID for
///   any state the search has encountered so far (via state expansions or
///   setting the start or goal) changes, the search will be invalid.
///
/// * Changes to the goal state are reflected by changes to the goal state ID.
///   Often, many graph representations that support multiple or underdefined
///   goal states will represent the goal state given to the planner using a
///   single goal state ID. If this is the case, the caller will have to assert
///   whether or not the goal has changed, and force the planner to reinitialize
///   by calls for force_planning_from_scratch (TODO: shouldn't require full
///   reinitialization)
///
/// * The heuristics for any encountered states remain constant, unless the goal
///   state ID has changed.

enum Modes
{
    PRECOMPUTE_PATH = 0,
    REACHABILITY,
    QUERY
};

class ARAStarZero : public SBPLPlanner
{
public:

    // parameters for controlling how long the search runs
    struct TimeParameters
    {
        bool bounded;
        bool improve;
        enum TimingType { EXPANSIONS, TIME } type;
        int max_expansions_init;
        int max_expansions;
        clock::duration max_allowed_time_init;
        clock::duration max_allowed_time;
    };

    struct RejectionStats
    {
        int cc;
        int ik;
        int reg;
    }rejection_stats;


    ARAStarZero(DiscreteSpaceInformation* space, Heuristic* heuristic);
    ~ARAStarZero();

    void allowPartialSolutions(bool enabled) {
        m_allow_partial_solutions = enabled;
    }

    bool allowPartialSolutions() const { return m_allow_partial_solutions; }

    void setAllowedRepairTime(double allowed_time_secs) {
        m_time_params.max_allowed_time = to_duration(allowed_time_secs);
    }

    double allowedRepairTime() const {
        return to_seconds(m_time_params.max_allowed_time);
    }

    void setTargetEpsilon(double target_eps) {
        m_final_eps = std::max(target_eps, 1.0);
    }

    double targetEpsilon() const { return m_final_eps; }

    void setDeltaEpsilon(double delta_eps) {
        assert(delta_eps > 0.0);
        m_delta_eps = delta_eps;
    }

    double deltaEpsilon() const { return m_delta_eps; }

    void setImproveSolution(bool improve) {
        m_time_params.improve = improve;
    }

    bool improveSolution() const { return m_time_params.improve; }

    void setBoundExpansions(bool bound) { m_time_params.bounded = bound; }
    bool boundExpansions() const { return m_time_params.bounded; }

    int replan(
        const TimeParameters &params,
        std::vector<int>* solution,
        int* cost);

    /// \name Required Functions from SBPLPlanner
    ///@{
    int replan(double allowed_time_secs, std::vector<int>* solution) override;
    int replan(double allowed_time_secs, std::vector<int>* solution, int* solcost) override;
    int set_goal(int state_id) override;
    int set_start(int state_id) override;
    int force_planning_from_scratch() override;
    int set_search_mode(bool bSearchUntilFirstSolution) override;
    void costs_changed(const StateChangeQuery& stateChange) override;
    ///@}

    /// \name Reimplemented Functions from SBPLPlanner
    ///@{
    int replan(std::vector<int>* solution, ReplanParams params) override;
    int replan(std::vector<int>* solution, ReplanParams params, int* solcost) override;
    int force_planning_from_scratch_and_free_memory() override;
    double get_solution_eps() const override;
    int get_n_expands() const override;
    double get_initial_eps() override;
    double get_initial_eps_planning_time() override;
    double get_final_eps_planning_time() override;
    int get_n_expands_init_solution() override;
    double get_final_epsilon() override;
    void get_search_stats(std::vector<PlannerStats>* s) override;
    void set_initialsolution_eps(double eps) override;

    unsigned int compute_reachability(const unsigned int r_max, int attractor_state_id);
    void update_search_mode(int mode);
    ///@}

private:

    struct SearchState : public heap_element
    {
        int state_id;       // corresponding graph state
        unsigned int g;     // cost-to-come
        unsigned int h;     // estimated cost-to-go
        unsigned int f;     // (g + eps * h) at time of insertion into OPEN
        unsigned int eg;    // g-value at time of expansion
        unsigned short iteration_closed;
        unsigned short call_number;
        SearchState* bp;
        bool incons;
        bool greedy;
    };

    struct SearchStateCompare
    {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            return s1.f < s2.f;
        }
    };

    DiscreteSpaceInformation* m_space;
    DiscreteSpaceInformation* m_manip_space;
    // sbpl::motion::ManipLattice* m_task_space;
    sbpl::motion::WorkspaceLatticeZero* m_task_space;
    Heuristic* m_heur;
    Heuristic* m_heur2;

    TimeParameters m_time_params;

    double m_initial_eps;
    double m_final_eps;
    double m_delta_eps;

    bool m_allow_partial_solutions;

    std::vector<SearchState*> m_states;

    int m_start_state_id;   // graph state id for the start state
    int m_goal_state_id;    // graph state id for the goal state

    // search state (not including the values of g, f, back pointers, and
    // closed list from m_stats)
    intrusive_heap<SearchState, SearchStateCompare> m_open;
    std::vector<SearchState*> m_incons;
    double m_curr_eps;
    int m_iteration;

    std::vector<int> m_succs;
    std::vector<int> m_preds;
    std::vector<int> m_costs;

    int m_call_number;          // for lazy reinitialization of search states
    int m_last_start_state_id;  // for lazy reinitialization of the search tree
    int m_last_goal_state_id;   // for updating the search tree when the goal changes
    double m_last_eps;          // for updating the search tree when heuristics change

    int m_expand_count_init;
    clock::duration m_search_time_init;
    int m_expand_count;
    clock::duration m_search_time;

    double m_satisfied_eps;

    // reachability
    int m_reachability_expansions;
    int m_next_seed_id;
    bool m_preprocessing;
    int m_search_mode;

    void convertTimeParamsToReplanParams(
        const TimeParameters& t,
        ReplanParams& r) const;
    void convertReplanParamsToTimeParams(
        const ReplanParams& r,
        TimeParameters& t);

    bool timedOut(
        int elapsed_expansions,
        const clock::duration& elapsed_time) const;

    int improvePath(
        const clock::time_point& start_time,
        SearchState* goal_state,
        int& elapsed_expansions,
        clock::duration& elapsed_time);

    void expand(SearchState* s);

    void recomputeHeuristics();
    void reorderOpen();
    int computeKey(SearchState* s) const;

    SearchState* getSearchState(int state_id);
    SearchState* createState(int state_id);
    void reinitSearchState(SearchState* state);

    void extractPath(
        SearchState* to_state,
        std::vector<int>& solution,
        int& cost) const;
};

} // namespace sbpl

#endif
