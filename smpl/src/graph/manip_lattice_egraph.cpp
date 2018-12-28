////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#include <smpl/graph/manip_lattice_egraph.h>

// standard includes
#include <fstream>

// system includes
#include <boost/filesystem.hpp>

// project includes
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/csv_parser.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heap/intrusive_heap.h>

namespace smpl {

auto ManipLatticeEgraph::RobotCoordHash::operator()(const argument_type& s) const ->
    result_type
{
    auto seed = (size_t)0;
    boost::hash_combine(seed, boost::hash_range(s.begin(), s.end()));
    return seed;
}

static
bool findShortestExperienceGraphPath(
    ManipLatticeEgraph* lattice,
    ExperienceGraph::node_id start_node,
    ExperienceGraph::node_id goal_node,
    std::vector<ExperienceGraph::node_id>& path)
{
    struct ExperienceGraphSearchNode : heap_element
    {
        int g;
        bool closed;
        ExperienceGraphSearchNode* bp;
        ExperienceGraphSearchNode() :
            g(std::numeric_limits<int>::max()),
            closed(false),
            bp(nullptr)
        { }
    };

    struct NodeCompare
    {
        bool operator()(
            const ExperienceGraphSearchNode& a,
            const ExperienceGraphSearchNode& b)
        {
            return a.g < b.g;
        }
    };

    using heap_type = intrusive_heap<ExperienceGraphSearchNode, NodeCompare>;

    auto search_nodes = std::vector<ExperienceGraphSearchNode>(lattice->m_egraph.num_nodes());

    auto open = heap_type();

    search_nodes[start_node].g = 0;
    open.push(&search_nodes[start_node]);
    auto exp_count = 0;
    while (!open.empty()) {
        ++exp_count;
        auto* min = open.min();
        open.pop();
        min->closed = true;

        if (min == &search_nodes[goal_node]) {
            SMPL_ERROR("Found shortest experience graph path");
            ExperienceGraphSearchNode* ps = nullptr;
            for (ExperienceGraphSearchNode* s = &search_nodes[goal_node];
                s; s = s->bp)
            {
                if (s != ps) {
                    path.push_back(std::distance(search_nodes.data(), s));
                    ps = s;
                } else {
                    SMPL_ERROR("Cycle detected!");
                }
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        auto n = (ExperienceGraph::node_id)std::distance(search_nodes.data(), min);
        auto adj = lattice->m_egraph.adjacent_nodes(n);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            auto& succ = search_nodes[*ait];
            if (succ.closed) {
                continue;
            }
            auto new_cost = min->g + 1;
            if (new_cost < succ.g) {
                succ.g = new_cost;
                succ.bp = min;
                if (open.contains(&succ)) {
                    open.decrease(&succ);
                } else {
                    open.push(&succ);
                }
            }
        }
    }

    SMPL_INFO("Expanded %d nodes looking for shortcut", exp_count);
    return false;
}

static
bool parseExperienceGraphFile(
    const ManipLatticeEgraph* lattice,
    const std::string& filepath,
    std::vector<RobotState>& egraph_states)
{
    auto fin = std::ifstream(filepath);
    if (!fin.is_open()) {
        return false;
    }

    auto parser = CSVParser();
    auto with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        SMPL_ERROR("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    SMPL_INFO("Parsed experience graph file");
    SMPL_INFO("  Has Header: %s", parser.hasHeader() ? "true" : "false");
    SMPL_INFO("  %zu records", parser.recordCount());
    SMPL_INFO("  %zu fields", parser.fieldCount());

    auto jvar_count = lattice->robot()->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count) {
        SMPL_ERROR("Parsed experience graph contains insufficient number of joint variables");
        return false;
    }

    egraph_states.reserve(parser.totalFieldCount());
    for (auto i = 0; i < parser.recordCount(); ++i) {
        auto state = RobotState(jvar_count);
        for (auto j = 0; j < parser.fieldCount(); ++j) {
            try {
                state[j] = std::stod(parser.fieldAt(i, j));
            } catch (const std::invalid_argument& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            } catch (const std::out_of_range& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            }
        }
        egraph_states.push_back(std::move(state));
    }

    SMPL_INFO("Read %zu states from experience graph file", egraph_states.size());
    return true;
}

int ManipLatticeEgraph::getStartStateID() const
{
    return this->lattice.getStartStateID();
}

int ManipLatticeEgraph::getGoalStateID() const
{
    return this->lattice.getGoalStateID();
}

bool ManipLatticeEgraph::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << idpath);
    if (idpath.empty()) {
        return true;
    }

    assert(idpath.size() > 1);
    assert(idpath[0] != getGoalStateID());

    auto opath = std::vector<RobotState>();

    // grab the first point
    {
        assert(IsValidStateID(this, idpath[0]));
        auto* first_state = lattice.getHashEntry(idpath[0]);
        if (!first_state) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(first_state->state);
    }

    // grab the rest of the points
    for (auto i = 1; i < idpath.size(); ++i) {
        auto prev_id = idpath[i - 1];
        auto curr_id = idpath[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        assert(prev_id != getGoalStateID());

        auto* prev_state = lattice.getHashEntry(prev_id);

        auto best_action = lattice.FindBestAction(prev_id, curr_id);
        if (best_action >= 0) {
            auto best_action_path = lattice.m_action_space->GetActionPath(prev_id, prev_state, best_action);
            for (auto& wp : best_action_path) {
                opath.push_back(wp);
            }
            continue;
        }

        // check for shortcut transition
        auto pnit = std::find(m_egraph_state_ids.begin(), m_egraph_state_ids.end(), prev_id);
        auto cnit = std::find(m_egraph_state_ids.begin(), m_egraph_state_ids.end(), curr_id);
        if (pnit != m_egraph_state_ids.end() &&
            cnit != m_egraph_state_ids.end())
        {
            auto pn = (ExperienceGraph::node_id)std::distance(m_egraph_state_ids.begin(), pnit);
            auto cn = (ExperienceGraph::node_id)std::distance(m_egraph_state_ids.begin(), cnit);

            SMPL_INFO("Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, pn, cn);

            auto node_path = std::vector<ExperienceGraph::node_id>();
            if (findShortestExperienceGraphPath(this, pn, cn, node_path)) {
                for (auto n : node_path) {
                    auto state_id = m_egraph_state_ids[n];
                    auto* entry = lattice.getHashEntry(state_id);
                    assert(entry != NULL);
                    opath.push_back(entry->state);
                }
                continue;
            }
        }

        // check for snap transition
        SMPL_DEBUG_NAMED(G_LOG, "Check for snap successor");
        int cost;
        if (snap(prev_id, curr_id, cost)) {
            SMPL_ERROR("Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            auto* entry = lattice.getHashEntry(curr_id);
            assert(entry);
            opath.push_back(entry->state);
            continue;
        }

        SMPL_ERROR_NAMED(G_LOG, "Failed to find valid goal successor during path extraction");
        return false;
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, lattice.getStateVisualization(path.back(), vis_name));
    return true;
}

void ManipLatticeEgraph::GetSuccs(int state_id, std::vector<int>* succs, std::vector<int>* costs)
{
    return this->lattice.GetSuccs(state_id, succs, costs);
}

void ManipLatticeEgraph::GetPreds(int state_id, std::vector<int>* preds, std::vector<int>* costs)
{
    return this->lattice.GetPreds(state_id, preds, costs);
}

void ManipLatticeEgraph::PrintState(int state_id, bool verbose, FILE* f)
{
    return this->lattice.PrintState(state_id, verbose, f);
}

bool ManipLatticeEgraph::projectToPose(int state_id, Affine3& pose)
{
    return this->lattice.projectToPose(state_id, pose);
}

auto ManipLatticeEgraph::extractState(int state_id) -> const RobotState&
{
    return this->lattice.extractState(state_id);
}

bool ManipLatticeEgraph::loadExperienceGraph(const std::string& path)
{
    SMPL_INFO("Load Experience Graph at %s", path.c_str());

    boost::filesystem::path p(path);
    if (!boost::filesystem::is_directory(p)) {
        SMPL_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        auto& filepath = dit->path().generic_string();
        std::vector<RobotState> egraph_states;
        if (!parseExperienceGraphFile(this, filepath, egraph_states)) {
            continue;
        }

        if (egraph_states.empty()) {
            continue;
        }

        SMPL_INFO("Create hash entries for experience graph states");

        auto& pp = egraph_states.front();  // previous robot state
        RobotCoord pdp(robot()->jointVariableCount()); // previous robot coord
        lattice.stateToCoord(egraph_states.front(), pdp);

        auto pid = m_egraph.insert_node(pp);
        m_coord_to_nodes[pdp].push_back(pid);

        int entry_id = lattice.reserveHashEntry();
        auto* entry = lattice.getHashEntry(entry_id);
        entry->coord = pdp;
        entry->state = pp;

        // map state id <-> experience graph state
        m_egraph_state_ids.resize(pid + 1, -1);
        m_egraph_state_ids[pid] = entry_id;
        m_state_to_node[entry_id] = pid;

        std::vector<RobotState> edge_data;
        for (size_t i = 1; i < egraph_states.size(); ++i) {
            auto& p = egraph_states[i];
            RobotCoord dp(robot()->jointVariableCount());
            lattice.stateToCoord(p, dp);
            if (dp != pdp) {
                // found a new discrete state along the path

                auto id = m_egraph.insert_node(p);
                m_coord_to_nodes[dp].push_back(id);

                int entry_id = lattice.reserveHashEntry();
                auto* entry = lattice.getHashEntry(entry_id);
                entry->coord = dp;
                entry->state = p;

                m_egraph_state_ids.resize(id + 1, -1);
                m_egraph_state_ids[id] = entry_id;
                m_state_to_node[entry_id] = id;
                m_egraph.insert_edge(pid, id, edge_data);

                pdp = dp;
                pid = id;
                edge_data.clear();
            } else {
                // gather intermediate robot states
                edge_data.push_back(p);
            }
        }
    }

    SMPL_INFO("Experience graph contains %zu nodes and %zu edges", m_egraph.num_nodes(), m_egraph.num_edges());
    return true;
}

void ManipLatticeEgraph::getExperienceGraphNodes(
    int state_id,
    std::vector<ExperienceGraph::node_id>& nodes)
{
    auto it = m_state_to_node.find(state_id);
    if (it != m_state_to_node.end()) {
        nodes.push_back(it->second);
    }
}

bool ManipLatticeEgraph::shortcut(
    int first_id,
    int second_id,
    int& cost)
{
    auto* first_entry = lattice.getHashEntry(first_id);
    auto* second_entry = lattice.getHashEntry(second_id);
    if (!first_entry | !second_entry) {
        SMPL_WARN("No state entries for state %d or state %d", first_id, second_id);
        return false;
    }

    SMPL_INFO_STREAM("Shortcut " << first_entry->state << " -> " << second_entry->state);
    auto* vis_name = "shortcut";
    SV_SHOW_INFO_NAMED(vis_name, lattice.getStateVisualization(first_entry->state, "shortcut_from"));
    SV_SHOW_INFO_NAMED(vis_name, lattice.getStateVisualization(second_entry->state, "shortcut_to"));

    SMPL_INFO("  Shortcut %d -> %d!", first_id, second_id);
    cost = 1000;
    return true;
}

bool ManipLatticeEgraph::snap(
    int first_id,
    int second_id,
    int& cost)
{
    auto* first_entry = lattice.getHashEntry(first_id);
    auto* second_entry = lattice.getHashEntry(second_id);
    if (!first_entry | !second_entry) {
        SMPL_WARN("No state entries for state %d or state %d", first_id, second_id);
        return false;
    }

    SMPL_INFO_STREAM("Snap " << first_entry->state << " -> " << second_entry->state);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, lattice.getStateVisualization(first_entry->state, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, lattice.getStateVisualization(second_entry->state, "snap_to"));

    if (!collisionChecker()->isStateToStateValid(
            first_entry->state, second_entry->state))
    {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_INFO("  Snap %d -> %d!", first_id, second_id);
    cost = 1000;
    return true;
}

const ExperienceGraph* ManipLatticeEgraph::getExperienceGraph() const
{
    return &m_egraph;
}

ExperienceGraph* ManipLatticeEgraph::getExperienceGraph()
{
    return &m_egraph;
}

int ManipLatticeEgraph::getStateID(ExperienceGraph::node_id n) const
{
    if (n >= m_egraph_state_ids.size()) {
        return -1;
    } else {
        return m_egraph_state_ids[n];
    }
}

auto ManipLatticeEgraph::getExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<ExperienceGraphExtension>() ||
        class_code == GetClassCode<ExtractRobotStateInterface>())
    {
        return this;
    }

    if (class_code == GetClassCode<PoseProjectionExtension>() &&
        lattice_pose_projection != NULL)
    {
        return this;
    }

    return NULL;
}

/// An attempt to construct the discrete experience graph by discretizing all
/// input continuous states and connecting them via edges available in the
/// canonical action set. This turns out to not work very well since the points
/// are not often able to be connected by the limited action set. It also
/// necessitates imposing restrictions on the action set, since context-specific
/// actions don't make sense before an actual planning request.
void rasterizeExperienceGraph()
{
//    std::vector<RobotCoord> egraph_coords;
//    for (const RobotState& state : egraph_states) {
//        RobotCoord coord(robot()->jointVariableCount());
//        lattice.stateToCoord(state, coord);
//        egraph_coords.push_back(std::move(coord));
//    }
//
//    auto it = std::unique(egraph_coords.begin(), egraph_coords.end());
//    egraph_coords.erase(it, egraph_coords.end());
//
//    SMPL_INFO("Experience contains %zu discrete states", egraph_coords.size());
//    for (const RobotCoord& coord : egraph_coords) {
//        SMPL_STREAM_INFO("  " << coord);
//    }
//
//    SMPL_INFO("Insert states into experience graph and map coords to experience graph nodes");
//
//    // insert all coords into egraph and initialize coord -> egraph node mapping
//    RobotState state(robot()->jointVariableCount());
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        coordToState(*it, state);
//        m_coord_to_nodes[*it] = m_egraph.insert_node(state);
//    }
//
//    SMPL_INFO("Insert experience graph edges into experience graph");
//
//    int edge_count = 0;
//    ManipLatticeActionSpace* aspace =
//            dynamic_cast<ManipLatticeActionSpace*>(actionSpace().get());
//    if (!aspace) {
//        SMPL_ERROR("ManipLatticeEgraph requires action space to be a ManipLatticeActionSpace");
//        return false;
//    }
//
//    // save action space configuration
//    bool mprim_enabled_state[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        mprim_enabled_state[i] = aspace->useAmp((MotionPrimitive::Type)i);
//    }
//    bool use_long_and_short_mprims = aspace->useLongAndShortPrims();
//
//    // disable context-specific motion primitives
//    aspace->useAmp(MotionPrimitive::SHORT_DISTANCE, true);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_RPY, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, false);
//
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        const ExperienceGraph::node_id n = m_coord_to_nodes[*it];
//        RobotState source(robot()->jointVariableCount());
//        coordToState(*it, source);
//        std::vector<Action> actions;
//        aspace->apply(source, actions);
//        SMPL_INFO("%zu actions from egraph state", actions.size());
//        for (const Action& action : actions) {
//            RobotCoord last(robot()->jointVariableCount());
//            lattice.stateToCoord(action.back(), last);
//            SMPL_INFO("Check for experience graph edge " << *it << " -> " << last);
//            auto iit = m_coord_to_nodes.find(last);
//            if (iit != m_coord_to_nodes.end() && !m_egraph.edge(n, iit->second)) {
//                m_egraph.insert_edge(n, iit->second);
//                ++edge_count;
//            }
//        }
//    }
//    SMPL_INFO("Experience graph contains %d edges", edge_count);
//
//    // restore action space configuration
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        aspace->useAmp((MotionPrimitive::Type)i, mprim_enabled_state[i]);
//    }
//    aspace->useLongAndShortPrims(use_long_and_short_mprims);
}

} // namespace smpl
