////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen
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

#ifndef SBPL_PR2_ROBOT_MODEL_UBR1_KDL_ROBOT_MODEL_H
#define SBPL_PR2_ROBOT_MODEL_UBR1_KDL_ROBOT_MODEL_H

// standard includes
#include <string>

// system includes
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

// project includes
#include <sbpl_pr2_robot_model/orientation_solver.h>

namespace sbpl {
namespace motion {

class UBR1KDLRobotModel : public KDLRobotModel
{
public:

    bool init(
        const std::string& robot_description,
        const std::string& base_link,
        const std::string& tip_link,
        int free_angle = DEFAULT_FREE_ANGLE_INDEX);

    bool computeIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED) override;

private:

    std::unique_ptr<RPYSolver> m_rpy_solver;

    std::string m_forearm_roll_link_name;
    std::string m_wrist_pitch_joint_name;
    std::string m_end_effector_link_name;
};

} // namespace motion
} // namespace sbpl

#endif
