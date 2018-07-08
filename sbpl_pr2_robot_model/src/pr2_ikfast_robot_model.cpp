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

#include <sbpl_pr2_robot_model/pr2_ikfast_robot_model.h>

// standard includes
#include <boost/make_shared.hpp>

// system includes
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <boost/make_shared.hpp>

using namespace std;

namespace sbpl {
namespace motion {

PR2IKFastRobotModel::PR2IKFastRobotModel()
{
}

PR2IKFastRobotModel::~PR2IKFastRobotModel()
{
}

bool PR2IKFastRobotModel::init(
    const std::string& robot_description,
    const std::vector<std::string>& planning_joints,
    const std::string& chain_root_link,
    const std::string& chain_tip_link,
    int free_angle)
{
    kinematics_frame_ = chain_root_link;
    chain_root_name_ = chain_root_link;
    chain_tip_name_ = chain_tip_link;
    free_angle_ = free_angle;

    ROS_INFO("Initialize KDL Robot Model");
    if (!urdf_.initString(robot_description)) {
        ROS_ERROR("Failed to parse the URDF.");
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(urdf_, ktree_)) {
        ROS_ERROR("Failed to parse the kdl tree from robot description.");
        return false;
    }

    std::vector<std::string> segments(planning_joints.size());
    for (size_t j = 0; j < planning_joints.size(); ++j) {
        if (!leatherman::getSegmentOfJoint(ktree_, planning_joints[j], segments[j])) {
            ROS_ERROR("Failed to find kdl segment for '%s'.", planning_joints_[j].c_str());
            return false;
        }
    }

    if (!ktree_.getChain(chain_root_name_, chain_tip_name_, kchain_)) {
        ROS_ERROR("Failed to fetch the KDL chain for the robot. (root: %s, tip: %s)", chain_root_name_.c_str(), chain_tip_name_.c_str());
        return false;
    }

    // check if our chain includes all planning joints
    for (auto& planning_joint : planning_joints) {
        int index;
        if (!leatherman::getJointIndex(kchain_, planning_joint, index)) {
            ROS_ERROR("Failed to find '%s' in the kinematic chain. Maybe your chain root or tip joints are wrong? (%s, %s)", planning_joint.c_str(), chain_root_name_.c_str(), chain_tip_name_.c_str());
            return false;
        }
    }

    // joint limits
    planning_joints_ = planning_joints;
    if (!getJointLimits(
                planning_joints_,
                min_limits_,
                max_limits_,
                continuous_,
                vel_limits_,
                eff_limits_))
    {
        ROS_ERROR("Failed to get the joint limits.");
        return false;
    }

    ROS_INFO("Min Limits: %s", to_string(min_limits_).c_str());
    ROS_INFO("Max Limits: %s", to_string(max_limits_).c_str());
    ROS_INFO("Continuous: %s", to_string(continuous_).c_str());

    // FK solver
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kchain_));
    jnt_pos_in_.resize(kchain_.getNrOfJoints());
    jnt_pos_out_.resize(kchain_.getNrOfJoints());

    // IK solver
    KDL::JntArray q_min(planning_joints_.size());
    KDL::JntArray q_max(planning_joints_.size());
    for (size_t i = 0; i < planning_joints_.size(); ++i) {
        q_min(i) = min_limits_[i];
        q_max(i) = max_limits_[i];
    }
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kchain_));

    const int max_iterations = 200;
    const double kdl_eps = 0.001;
    ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(
            kchain_, q_min, q_max, *fk_solver_, *ik_vel_solver_, max_iterations, kdl_eps));

    // joint name -> index mapping
    for (size_t i = 0; i < planning_joints_.size(); ++i) {
        joint_map_[planning_joints_[i]] = i;
    }

    // link name -> kdl index mapping
    for (size_t i = 0; i < kchain_.getNrOfSegments(); ++i) {
        link_map_[kchain_.getSegment(i).getName()] = i;
    }

    initialized_ = true;

    ROS_INFO("Initialize PR2 IKFast Robot Model");

    m_robot_loader = boost::make_shared<robot_model_loader::RobotModelLoader>(
            "robot_description", true);
    m_robot_model = m_robot_loader->getModel();
    if (!m_robot_model) {
        ROS_ERROR("Robot model is null");
        return false;
    }

    m_robot_state = boost::make_shared<robot_state::RobotState>(m_robot_model);
    m_robot_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = m_robot_model->getJointModelGroup("right_arm");
    return true;
}

bool PR2IKFastRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const RobotState& seed,
    RobotState& solution)
{
    const robot_state::JointModelGroup* joint_model_group = m_robot_model->getJointModelGroup("right_arm");
    m_robot_state->setJointGroupPositions(joint_model_group, seed);

    kinematics::KinematicsQueryOptions options;
    options.lock_redundant_joints = true;
    if (!m_robot_state->setFromIK(
            joint_model_group,
            pose,
            1,
            0.1,
            moveit::core::GroupStateValidityCallbackFn(),
            options)) {
        return false;
    }
    m_robot_state->copyJointGroupPositions(joint_model_group, solution);
    return true;
}

Extension* PR2IKFastRobotModel::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotModel>() ||
    class_code == GetClassCode<ForwardKinematicsInterface>() ||
    class_code == GetClassCode<InverseKinematicsInterface>() ||
    class_code == GetClassCode<RedundantManipulatorInterface>()) {
        return this;
    }

    return nullptr;
}

} // namespace motion
} // namespace sbpl
