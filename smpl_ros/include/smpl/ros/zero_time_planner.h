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

#ifndef SMPL_ZERO_TIME_PLANNER_H
#define SMPL_ZERO_TIME_PLANNER_H

// system includes
#include <ros/ros.h>

// project includes
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/workspace_lattice_zero.h>

#include <smpl/search/arastar.h>
#include <smpl/search/arastar_zero.h>
#include <smpl/types.h>

namespace sbpl {
namespace motion {


class ZeroTimePlanner
{
public:

	ZeroTimePlanner(
        const RobotState& start_state,
        const GoalConstraint& goal,
		ManipLattice* manip_space,
		WorkspaceLatticeZero* task_space,
		ARAStar* planner,
	    ARAStarZero* planner_zero);

	~ZeroTimePlanner();

    void PreProcess();
    void Query(std::vector<RobotState>& path);

private:

    const GoalConstraint m_goal;
    const RobotState m_start_state;

    ManipLattice* m_manip_space;
    WorkspaceLatticeZero* m_task_space;

    ARAStar* m_planner;
    ARAStarZero* m_planner_zero;

    std::vector<region> m_regions;

    void WriteRegions();
    void ReadRegions();
};

} // namespace motion
} // namespace sbpl

#endif