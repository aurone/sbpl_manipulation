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

#ifndef SBPL_KDL_ROBOT_MODEL_KDL_ROBOT_MODEL_H
#define SBPL_KDL_ROBOT_MODEL_KDL_ROBOT_MODEL_H

// standard includes
#include <memory>
#include <string>

// system includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <smpl/robot_model.h>
#include <smpl_urdf_robot_model/smpl_urdf_robot_model.h>
#include <urdf/model.h>

namespace sbpl {
namespace motion {

class KDLRobotModel :
    public virtual urdf::URDFRobotModel,
    public virtual InverseKinematicsInterface,
    public virtual RedundantManipulatorInterface
{
public:

    static const int DEFAULT_FREE_ANGLE_INDEX = 2;

    bool init(
        const std::string& robot_description,
        const std::string& base_link,
        const std::string& tip_link,
        int free_angle = DEFAULT_FREE_ANGLE_INDEX);

    auto getBaseLink() const -> const std::string&;
    auto getPlanningLink() const -> const std::string&;

    bool computeIKSearch(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution);

    void printRobotModelInformation();

    /// \name RedundantManipulatorInterface
    /// @{
    const int redundantVariableCount() const override { return 0; }
    const int redundantVariableIndex(int vidx) const override { return 0.0; }
    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution) override;
    /// @}

    /// \name InverseKinematicsInterface Interface
    ///@{
    bool computeIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED) override;

    bool computeIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        std::vector<RobotState>& solutions,
        ik_option::IkOption option = ik_option::UNRESTRICTED) override;
    ///@}

    /// \name Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> Extension* override;
    ///@}

public:

    ::urdf::Model m_urdf;

    urdf::RobotModel m_robot_model;

    const urdf::Link* m_kinematics_link = NULL;

    std::string m_base_link;
    std::string m_tip_link;

    KDL::Tree m_tree;
    KDL::Chain m_chain;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive>    m_fk_solver;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv>         m_ik_vel_solver;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>        m_ik_solver;

    // ik solver settings
    int m_max_iterations;
    double m_kdl_eps;

    // temporary storage
    KDL::JntArray m_jnt_pos_in;
    KDL::JntArray m_jnt_pos_out;

    // ik search configuration
    int m_free_angle;
    double m_search_discretization;
    double m_timeout;
};

} // namespace motion
} // namespace sbpl

#endif
