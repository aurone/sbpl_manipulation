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

#ifndef SBPL_IKFAST_ROBOT_MODEL_H
#define SBPL_IKFAST_ROBOT_MODEL_H

// standard includes


// system includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/console.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/robot_model.h>

// project includes


namespace sbpl {
namespace motion {

class PR2IKFastRobotModel :
    public KDLRobotModel,
    public virtual RedundantManipulatorInterface
{
public:

    static const int DEFAULT_FREE_ANGLE_INDEX = 0;

    PR2IKFastRobotModel();

    ~PR2IKFastRobotModel();

    /// \name Reimplemented Functions from IKFastRobotModel
    ///@{
    bool init(
        const std::string& robot_description,
        const std::vector<std::string>& planning_joints,
        const std::string& chain_root_link,
        const std::string& chain_tip_link,
        int free_angle = DEFAULT_FREE_ANGLE_INDEX) override;

    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const RobotState& seed,
        RobotState& solution) override;
    ///@}

    /// \name Extension Interface
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    const int redundantVariableCount() const override {return 1;};
    const int redundantVariableIndex(int rvidx) const override {return 0;};

private:

    robot_model_loader::RobotModelLoaderPtr m_robot_loader;
    moveit::core::RobotModelPtr             m_robot_model;
    moveit::core::RobotStatePtr             m_robot_state;
};

} // namespace motion
} // namespace sbpl

#endif
