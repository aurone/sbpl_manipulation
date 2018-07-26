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

#include <sbpl_pr2_robot_model/pr2_kdl_robot_model.h>

// system includes
#include <kdl/tree.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <eigen_conversions/eigen_kdl.h>

namespace sbpl {
namespace motion {

bool PR2KDLRobotModel::init(
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    if (!KDLRobotModel::init(
            robot_description, base_link, tip_link, free_angle))
    {
        return false;
    }

    // PR2 Specific IK Solver
    pr2_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(m_urdf, base_link, tip_link, 0.02, 2));
    if (!pr2_ik_solver_->active_) {
        ROS_ERROR("The PR2 IK solver is NOT active.");
        return false;
    }

    // initialize rpy solver
    if (tip_link == "r_gripper_palm_link" || tip_link == "l_gripper_palm_link") {
        if (base_link.substr(0, 1) == "r") {
            m_forearm_roll_link_name = "r_forearm_roll_link";
            m_wrist_pitch_joint_name = "r_wrist_flex_joint";
            m_end_effector_link_name = "r_gripper_palm_link";
        } else {
            m_forearm_roll_link_name = "l_forearm_roll_link";
            m_wrist_pitch_joint_name = "l_wrist_flex_joint";
            m_end_effector_link_name = "l_gripper_palm_link";
        }

        auto* wrist_var = GetVariable(this->robot_model, &m_wrist_pitch_joint_name);
        if (wrist_var == NULL) {
            return false;
        }

        double wrist_min_limit;
        double wrist_max_limit;
        bool wrist_continuous;
        double wrist_vel_limit;
        double wrist_eff_limit;
        m_rpy_solver.reset(new RPYSolver(wrist_min_limit, wrist_max_limit));
    }

    return true;
}

static void NormalizeAngles(KDLRobotModel* model, KDL::JntArray* q)
{
    for (auto i = 0; i < model->jointVariableCount(); ++i) {
        if (model->vprops[i].continuous) {
            (*q)(i) = sbpl::angles::normalize_angle((*q)(i));
        }
    }
}

bool PR2KDLRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    auto* T_map_kinematics = GetLinkTransform(&this->robot_state, this->m_kinematics_link);
    KDL::Frame frame_des;
    tf::transformEigenToKDL(T_map_kinematics->inverse() * pose, frame_des);

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        m_jnt_pos_in(i) = start[i];
    }

    NormalizeAngles(this, &m_jnt_pos_in);

    if (option == ik_option::RESTRICT_XYZ && m_rpy_solver) {
        for (auto i = 0; i < this->getPlanningJoints().size(); ++i) {
            auto& var_name = this->getPlanningJoints()[i];
            auto* var = GetVariable(this->robot_model, &var_name);
            SetVariablePosition(&this->robot_state, var, start[i]);
        }

        auto* forearm_link = GetLink(this->robot_model, &m_forearm_roll_link_name);
        auto* end_effector_link = GetLink(this->robot_model, &m_end_effector_link_name);
        UpdateLinkTransform(&this->robot_state, forearm_link);
        UpdateLinkTransform(&this->robot_state, end_effector_link);

        auto* forearm_pose = GetLinkTransform(&this->robot_state, forearm_link);
        auto* end_effector_pose = GetLinkTransform(&this->robot_state, end_effector_link);

        KDL::Frame fpose;
        KDL::Frame epose;
        tf::transformEigenToKDL(*forearm_pose, fpose);
        tf::transformEigenToKDL(*end_effector_pose, epose);

        std::vector<double> vfpose(6, 0.0);
        vfpose[0] = fpose.p.x();
        vfpose[1] = fpose.p.y();
        vfpose[2] = fpose.p.z();
        fpose.M.GetRPY(vfpose[3], vfpose[4], vfpose[5]);

        std::vector<double> vepose(6, 0.0);
        vepose[0] = epose.p.x();
        vepose[1] = epose.p.y();
        vepose[2] = epose.p.z();
        epose.M.GetRPY(vepose[3], vepose[4], vepose[5]);

        std::vector<double> rpy(3, 0.0);
        frame_des.M.GetRPY(rpy[0], rpy[1], rpy[2]);
        solution.resize(start.size());
        return m_rpy_solver->computeRPYOnly(rpy, start, vfpose, vepose, 1, solution);
    }

    if (pr2_ik_solver_) {
        auto timeout = 0.2;
        auto consistency_limit = 2.0 * M_PI;
        if (pr2_ik_solver_->CartToJntSearch(
                m_jnt_pos_in,
                frame_des,
                m_jnt_pos_out,
                timeout,
                consistency_limit) < 0)
        {
            return false;
        }

        solution.resize(start.size());
        for (size_t i = 0; i < solution.size(); ++i) {
            solution[i] = m_jnt_pos_out(i);
        }
    }

    return KDLRobotModel::computeIK(pose, start, solution, option);
}

bool PR2KDLRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution)
{
    auto* T_map_kinematics = GetLinkTransform(&this->robot_state, this->m_kinematics_link);
    KDL::Frame frame_des;
    tf::transformEigenToKDL(T_map_kinematics->inverse() * pose, frame_des);

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        m_jnt_pos_in(i) = start[i];
    }

    NormalizeAngles(this, &m_jnt_pos_in);

    if (pr2_ik_solver_->CartToJnt(m_jnt_pos_in, frame_des, m_jnt_pos_out) < 0) {
        return false;
    }

    solution.resize(start.size());
    for (size_t i = 0; i < solution.size(); ++i) {
        solution[i] = m_jnt_pos_out(i);
    }

    return true;
}

} // namespace motion
} // namespace sbpl
