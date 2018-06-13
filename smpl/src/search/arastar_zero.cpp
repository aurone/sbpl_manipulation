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

#include <smpl/search/arastar_zero.h>

#include <algorithm>
#include <chrono>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>

namespace sbpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";
static const char* SRLOG = "search.reachability";
static const char* SRTLOG = "search.reachability.time";

ARAStarZero::ARAStarZero(
    DiscreteSpaceInformation* space,
    Heuristic* heur)
:
    SBPLPlanner(),
    m_space(space),
    m_heur(heur),
    m_time_params(),
    m_initial_eps(1.0),
    m_final_eps(1.0),
    m_delta_eps(1.0),
    m_allow_partial_solutions(false),
    m_states(),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_open(),
    m_incons(),
    m_curr_eps(1.0),
    m_iteration(1),
    m_call_number(0),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1),
    m_last_eps(1.0),
    m_expand_count_init(0),
    m_expand_count(0),
    m_search_time_init(clock::duration::zero()),
    m_search_time(clock::duration::zero()),
    m_satisfied_eps(std::numeric_limits<double>::infinity()),
    m_reachability_expansions(0),
    m_next_seed_id(-1),
    m_preprocessing(true)
{
    environment_ = space;

    // m_task_space = dynamic_cast<sbpl::motion::ManipLattice*>(space);
    m_task_space = dynamic_cast<sbpl::motion::WorkspaceLatticeZero*>(space);

    m_time_params.bounded = true;
    m_time_params.improve = false;
    m_time_params.type = TimeParameters::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
}

ARAStarZero::~ARAStarZero()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
}

enum ReplanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    TIMED_OUT,
    EXHAUSTED_OPEN_LIST
};

int ARAStarZero::replan(
    const TimeParameters& params,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    m_time_params = params;

    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    if (m_start_state_id != m_last_start_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Reinitialize search");
        m_open.clear();
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations

        reinitSearchState(start_state);
        reinitSearchState(goal_state);

        start_state->g = 0;
        start_state->f = computeKey(start_state);
        m_open.push(start_state);

        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;
    }

    if (m_goal_state_id != m_last_goal_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        recomputeHeuristics();
        reorderOpen();

        m_last_goal_state_id = m_goal_state_id;
    }

    auto start_time = clock::now();
    int num_expansions = 0;
    clock::duration elapsed_time = clock::duration::zero();

    int err;
    while (m_satisfied_eps > m_final_eps) {
        if (m_curr_eps == m_satisfied_eps) {
            if (!m_time_params.improve) {
                break;
            }
            // begin a new search iteration
            ++m_iteration;
            m_curr_eps -= m_delta_eps;
            m_curr_eps = std::max(m_curr_eps, m_final_eps);
            for (SearchState* s : m_incons) {
                s->incons = false;
                m_open.push(s);
            }
            reorderOpen();
            m_incons.clear();
            SMPL_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
        }
        err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
        if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
        }
        if (err) {
            break;
        }
        SMPL_DEBUG_NAMED(SLOG, "Improved solution");
        m_satisfied_eps = m_curr_eps;
    }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

    if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        if (m_allow_partial_solutions && !m_open.empty()) {
            SearchState* next_state = m_open.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        return !err;
    }

    extractPath(goal_state, *solution, *cost);
    return !SUCCESS;
}

int ARAStarZero::replan(
    double allowed_time,
    std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

// decide whether to start the search from scratch
//
// if start changed
//     reset the search to its initial state
// if goal changed
//     reevaluate heuristics
//     reorder the open list
//
// case scenario_hasnt_changed (start and goal the same)
//   case have solution for previous epsilon
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           pass
//   case dont have solution
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           reevaluate heuristics and reorder the open list
// case scenario_changed
int ARAStarZero::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    TimeParameters tparams = m_time_params;
    if (tparams.max_allowed_time_init == tparams.max_allowed_time) {
        // NOTE/TODO: this may lead to awkward behavior, if the caller sets the
        // allowed time to the current repair time, the repair time will begin
        // to track the allowed time for further calls to replan. perhaps set
        // an explicit flag for using repair time or an indicator value as is
        // done with ReplanParams
        tparams.max_allowed_time_init = to_duration(allowed_time);
        tparams.max_allowed_time = to_duration(allowed_time);
    } else {
        tparams.max_allowed_time_init = to_duration(allowed_time);
        // note: retain original allowed improvement time
    }
    return replan(tparams, solution, cost);
}

int ARAStarZero::replan(
    std::vector<int>* solution,
    ReplanParams params)
{
    int cost;
    return replan(solution, params, &cost);
}

int ARAStarZero::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* cost)
{
    // note: if replan fails before internal time parameters are updated (this
    // happens if the start or goal has not been set), then the internal
    // epsilons may be affected by this set of ReplanParams for future calls to
    // replan where ReplanParams is not used and epsilon parameters haven't been
    // set back to their desired values.
    TimeParameters tparams;
    convertReplanParamsToTimeParams(params, tparams);
    return replan(tparams, solution, cost);
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
int ARAStarZero::force_planning_from_scratch_and_free_memory()
{
    force_planning_from_scratch();
    m_open.clear();
    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
    m_states.clear();
    m_states.shrink_to_fit();
    return 0;
}

/// Return the suboptimality bound of the current solution for the current search.
double ARAStarZero::get_solution_eps() const
{
    return m_satisfied_eps;
}

/// Return the number of expansions made in progress to the final solution.
int ARAStarZero::get_n_expands() const
{
    return m_expand_count;
}

/// Return the initial suboptimality bound
double ARAStarZero::get_initial_eps()
{
    return m_initial_eps;
}

/// Return the time consumed by the search in progress to the initial solution.
double ARAStarZero::get_initial_eps_planning_time()
{
    return to_seconds(m_search_time_init);
}

/// Return the time consumed by the search in progress to the final solution.
double ARAStarZero::get_final_eps_planning_time()
{
    return to_seconds(m_search_time);
}

/// Return the number of expansions made in progress to the initial solution.
int ARAStarZero::get_n_expands_init_solution()
{
    return m_expand_count_init;
}

/// Return the final suboptimality bound.
double ARAStarZero::get_final_epsilon()
{
    return m_final_eps;
}

/// Return statistics for each completed search iteration.
void ARAStarZero::get_search_stats(std::vector<PlannerStats>* s)
{
    PlannerStats stats;
    stats.eps = m_curr_eps;
//    stats.cost; // TODO: implement
    stats.expands = m_expand_count;
    stats.time = to_seconds(m_search_time);
    s->push_back(stats);
}

/// Set the desired suboptimality bound for the initial solution.
void ARAStarZero::set_initialsolution_eps(double eps)
{
    m_initial_eps = eps;
}

/// Set the goal state.
int ARAStarZero::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

/// Set the start state.
int ARAStarZero::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

/// Force the search to forget previous search efforts and start from scratch.
int ARAStarZero::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    return 0;
}

/// Set whether the number of expansions is bounded by time or total expansions
/// per call to replan().
int ARAStarZero::set_search_mode(bool first_solution_unbounded)
{
    m_time_params.bounded = !first_solution_unbounded;
    return 0;
}

/// Notify the search of changes to edge costs in the graph.
void ARAStarZero::costs_changed(const StateChangeQuery& changes)
{
    force_planning_from_scratch();
}

// Recompute heuristics for all states.
void ARAStarZero::recomputeHeuristics()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            s->h = m_heur->GetGoalHeuristic(s->state_id);
        }
    }
}

// Convert TimeParameters to ReplanParams. Uses the current epsilon values
// to fill in the epsilon fields.
void ARAStarZero::convertTimeParamsToReplanParams(
    const TimeParameters& t,
    ReplanParams& r) const
{
    r.max_time = to_seconds(t.max_allowed_time_init);
    r.return_first_solution = !t.bounded && !t.improve;
    if (t.max_allowed_time_init == t.max_allowed_time) {
        r.repair_time = -1.0;
    } else {
        r.repair_time = to_seconds(t.max_allowed_time);
    }

    r.initial_eps = m_initial_eps;
    r.final_eps = m_final_eps;
    r.dec_eps = m_delta_eps;
}

// Convert ReplanParams to TimeParameters. Sets the current initial, final, and
// delta eps from ReplanParams.
void ARAStarZero::convertReplanParamsToTimeParams(
    const ReplanParams& r,
    TimeParameters& t)
{
    t.type = TimeParameters::TIME;

    t.bounded = !r.return_first_solution;
    t.improve = !r.return_first_solution;

    t.max_allowed_time_init = to_duration(r.max_time);
    if (r.repair_time > 0.0) {
        t.max_allowed_time = to_duration(r.repair_time);
    } else {
        t.max_allowed_time = t.max_allowed_time_init;
    }

    m_initial_eps = r.initial_eps;
    m_final_eps = r.final_eps;
    m_delta_eps = r.dec_eps;
}

// Test whether the search has run out of time.
bool ARAStarZero::timedOut(
    int elapsed_expansions,
    const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimeParameters::EXPANSIONS:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= m_time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= m_time_params.max_expansions;
        }
    case TimeParameters::TIME:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= m_time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= m_time_params.max_allowed_time;
        }
    default:
        SMPL_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}

unsigned int ARAStarZero::compute_reachability(const unsigned int r_max, int attractor_state_id)
{
    ++m_call_number;
    m_iteration = 1;
    SMPL_DEBUG_NAMED(SRLOG, "Reinitialize search");
    m_open.clear();

    auto attractor_state = getSearchState(attractor_state_id);
    reinitSearchState(attractor_state);
    attractor_state->greedy = true;

    m_preds.clear();
    m_costs.clear();
    m_task_space->GetPreds(attractor_state_id, &m_preds, &m_costs);
    for (const auto& pred_id : m_preds) {
        auto pred_state = getSearchState(pred_id);
        reinitSearchState(pred_state);
        pred_state->f = computeKey(pred_state);
        if (m_open.contains(pred_state)) {
            m_open.decrease(pred_state);
        } else {
            m_open.push(pred_state);
        }
    }

    unsigned int radius = 0;

    while (radius <= r_max && !m_open.empty()) {
        // getchar();
        m_succs.clear();
        m_costs.clear();
        SearchState* min_state = m_open.min();
        m_open.pop();
        assert(min_state->iteration_closed != m_iteration);
        assert(min_state->g != INFINITECOST);
        min_state->iteration_closed = m_iteration;

        SMPL_DEBUG_NAMED(SRLOG, "Expanding state: %d, h: %d", min_state->state_id, min_state->h);
        m_reachability_expansions++;

        ///@{ Greedy successor --line 7
        auto start = std::chrono::system_clock::now();
        m_task_space->GetSuccs(min_state->state_id, &m_succs, &m_costs);

        SMPL_DEBUG_NAMED(SRLOG, "Got %zu successors", m_succs.size());
        SearchState* succ_state_g;
        int min_h = INFINITECOST;
        std::vector<SearchState*> greedy_succs;

        for (const auto& succ_id : m_succs) {
            SearchState* succ_state = getSearchState(succ_id);
            reinitSearchState(succ_state);
            SMPL_DEBUG_NAMED(SRLOG, "    succ_id: %d, h: %d", succ_id, succ_state->h);
            if (succ_state->h < min_h) {
                min_h = succ_state->h;
                succ_state_g = succ_state;
            }
        }
#if 0   // tie breaking
        for (const auto& succ_id : m_succs) {
            SearchState* succ_state = getSearchState(succ_id);
            if (succ_state->h == min_h) {
                greedy_succs.push_back(succ_state);
                // printf("pushed back %d\n", succ_state->state_id);
            }
        }

        for (const auto& s : greedy_succs) {
            // printf("checking for %d\n", s->state_id);
            if (m_task_space->IsStateToStateValid(min_state->state_id, s->state_id)) {
                succ_state_g = s;
                break;
            }
        }
#endif
        std::chrono::duration<double, std::micro> duration = std::chrono::system_clock::now() - start;
        SMPL_DEBUG_NAMED(SRTLOG, "line 7: %f", duration.count());

        SMPL_DEBUG_NAMED(SRLOG, "Greedy succ: %d, h: %d", succ_state_g->state_id, min_h);
        ///@}

        ///@{ Greedy set criteria --line 8-9
        start = std::chrono::system_clock::now();
        if (succ_state_g->greedy &&
            m_task_space->IsStateToStateValid(min_state->state_id, succ_state_g->state_id)) {
                SMPL_DEBUG_NAMED(SRLOG, "State: %d added as greedy", min_state->state_id);
                min_state->greedy = true;
        }
        ///@}

        ///@{ Terminating condition --line 10-11 
        else if (m_task_space->IsStateValid(min_state->state_id)){
            SMPL_DEBUG_NAMED(SRLOG, "Exited obstacle hence terminating");
            m_next_seed_id = min_state->state_id;
            break;
        }
        else {
            SMPL_DEBUG_NAMED(SRLOG, "State invalid: entering obstacle");
        }
        duration = std::chrono::system_clock::now() - start;
        SMPL_DEBUG_NAMED(SRTLOG, "line 8-11: %f", duration.count());
        ///@}

        ///@{ Set radius --line 12
        radius = min_state->h;

        SMPL_DEBUG_NAMED(SRLOG, "Radius so far: %d", radius);
        ///@}

        ///@{ Insert Preds in Open list --line 13
        start = std::chrono::system_clock::now();
        SMPL_DEBUG_NAMED(SRLOG, "Inserting preds in Open");
        for (const auto& pred_id : m_succs) {   // because preds == succs
            // if (m_greedy.find(pred_id) == m_greedy.end()) {
            auto pred_state = getSearchState(pred_id);
            reinitSearchState(pred_state);
            if (!pred_state->greedy) {
                if (pred_state->iteration_closed != m_iteration) {
                    pred_state->f = computeKey(pred_state);
                    if (m_open.contains(pred_state)) {
                        m_open.decrease(pred_state);
                    } else {
                        SMPL_DEBUG_NAMED(SRLOG, "    pred_id %d", pred_id);
                        m_open.push(pred_state);
                    }
                }
            }
        }
        duration = std::chrono::system_clock::now() - start;
        SMPL_DEBUG_NAMED(SRTLOG, "line 13: %f", duration.count());
        ///@}

        SMPL_DEBUG_NAMED(SRLOG, "----------------------------------------");
    }
    /// line 14
    return radius;
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
int ARAStarZero::improvePath(
    const clock::time_point& start_time,
    SearchState* goal_state,
    int& elapsed_expansions,
    clock::duration& elapsed_time)
{
    while (!m_open.empty()) {
        SearchState* min_state = m_open.min();

        auto now = clock::now();
        elapsed_time = now - start_time;

        // path to goal found
        if (min_state->f >= goal_state->f || min_state == goal_state) {
            SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
            return SUCCESS;
        }

        if (timedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }

        SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

        m_open.pop();

        assert(min_state->iteration_closed != m_iteration);
        assert(min_state->g != INFINITECOST);

        min_state->iteration_closed = m_iteration;
        min_state->eg = min_state->g;

        expand(min_state);

        ++elapsed_expansions;
    }

    return EXHAUSTED_OPEN_LIST;
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and INCONS list appropriately.
void ARAStarZero::expand(SearchState* s)
{
    ROS_INFO("\nexpanding state: %d with h: %d", s->state_id, s->h);
    getchar();

    m_succs.clear();
    m_costs.clear();
    m_space->GetSuccs(s->state_id, &m_succs, &m_costs);

    SMPL_DEBUG_NAMED(SELOG, "  %zu successors", m_succs.size());

    for (size_t sidx = 0; sidx < m_succs.size(); ++sidx) {
        int succ_state_id = m_succs[sidx];
        int cost = m_costs[sidx];

        SearchState* succ_state = getSearchState(succ_state_id);
        reinitSearchState(succ_state);

        // ROS_INFO("    succ %d with h %d", succ_state_id, succ_state->h);

        int new_cost = s->eg + cost;
        SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        if (new_cost < succ_state->g) {
            succ_state->g = new_cost;
            succ_state->bp = s;
            if (succ_state->iteration_closed != m_iteration) {
                succ_state->f = computeKey(succ_state);
                if (m_open.contains(succ_state)) {
                    m_open.decrease(succ_state);
                } else {
                    m_open.push(succ_state);
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }
        }
    }
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void ARAStarZero::reorderOpen()
{
    for (auto it = m_open.begin(); it != m_open.end(); ++it) {
        (*it)->f = computeKey(*it);
    }
    m_open.make();
}

int ARAStarZero::computeKey(SearchState* s) const
{
    return (unsigned int)(m_curr_eps * s->h);
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
ARAStarZero::SearchState* ARAStarZero::getSearchState(int state_id)
{
    if (m_states.size() <= state_id) {
        m_states.resize(state_id + 1, nullptr);
    }

    auto& state = m_states[state_id];
    if (state == NULL) {
        state = createState(state_id);
    }

    return state;
}

// Create a new search state for a graph state.
ARAStarZero::SearchState* ARAStarZero::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState* ss = new SearchState;
    ss->state_id = state_id;
    ss->call_number = 0;

    return ss;
}

// Lazily (re)initialize a search state.
void ARAStarZero::reinitSearchState(SearchState* state)
{
    if (state->call_number != m_call_number) {
        SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        state->h = m_heur->GetGoalHeuristic(state->state_id);
        state->f = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
        state->incons = false;
        state->greedy = false;
    }
}

// Extract the path from the start state up to a new state.
void ARAStarZero::extractPath(
    SearchState* to_state,
    std::vector<int>& solution,
    int& cost) const
{
    for (SearchState* s = to_state; s; s = s->bp) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

} // namespace sbpl
