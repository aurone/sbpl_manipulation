////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush, Fahad Islam
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

#ifndef SMPL_WORKSPACE_LATTICE_ZERO_H
#define SMPL_WORKSPACE_LATTICE_ZERO_H

// standard includes
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/serialization/vector.hpp>

// project includes
#include <smpl/graph/workspace_lattice.h>

namespace sbpl {
namespace motion {

struct region
{
    friend class boost::serialization::access;
    WorkspaceLattice x;
    unsigned int radius;
    RobotState state;
    std::vector<RobotState> path;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & radius;
        ar & state;
        ar & path;
    }
};

} // namespace motion
} // namespace sbpl


namespace sbpl {
namespace motion {

/// \class Discrete state lattice representation representing a robot as the
///     pose of one of its links and all redundant joint variables
class WorkspaceLatticeZero :
    public WorkspaceLattice,
    public ExtractRobotStateExtension
{
public:

    enum Modes
    {
        REACHABILITY = 0,
        QUERY
    };

    ~WorkspaceLatticeZero();

    /// \name Reimplemented Public Functions from WorkspaceLatticeBase
    ///@{
    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const PlanningParams* pp,
        const Params& params) override;
    ///@}

    void ClearStates();

    // /// \name Reimplemented Public Functions from RobotPlanningSpace
    // ///@{
    bool setGoal(const GoalConstraint& goal) override;
    // ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;

    /// \name Required Public Functions from ExtractRobotStateExtension
    ///@{
    const RobotState& extractState(int state_id);
    ///@}

    // reachability
    bool IsStateValid(int state_id);
    bool IsStateToStateValid(int from_state_id, int to_state_id);
    void PruneRegions(std::vector<region>& regions);

    int SampleAttractorState(
        const std::vector<region>& regions,
        RobotState& robot_state,
        int max_tries);

    bool SampleState(RobotState& state);

    int FindRegionContainingState(const RobotState& state, std::vector<region> regions);

    bool IsStateInStartRegion(WorkspaceState state);

    void UpdateSearchMode(int search_mode){m_search_mode = search_mode;};

private:

    // reachability
    std::vector<double> m_min_ws_limits;
    std::vector<double> m_max_ws_limits;
    std::vector<std::uniform_real_distribution<double>> m_distribution;
    std::default_random_engine m_generator;
    GoalConstraint m_goal;
    WorkspaceState m_ws_state;
    RobotState m_ik_seed;

    //query
    int m_vis_id = 0;
    int m_search_mode;

    bool initMotionPrimitives();

    bool checkActionPreprocessing(
        const RobotState& state,
        const Action& action,
        RobotState* final_rstate = nullptr);

    bool checkAction(
        const RobotState& state,
        const Action& action,
        RobotState* final_rstate = nullptr);

    // used in path extraction
    bool isGoal(const WorkspaceState& state) const;
};

} // namespace motion
} // namespace sbpl

#endif

