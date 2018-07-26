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

#include <sbpl_kdl_robot_model/kdl_robot_model.h>

// system includes
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/time.h>

namespace sbpl {
namespace motion {

template <class T, class... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T> {
    return std::unique_ptr<T>(new T(args...));
}

static
bool getCount(int& count, int max_count, int min_count)
{
    if (count > 0) {
        if (-count >= min_count) {
            count = -count;
            return true;
        } else if (count + 1 <= max_count) {
            count = count + 1;
            return true;
        } else {
            return false;
        }
    } else {
        if (1 - count <= max_count) {
            count = 1 - count;
            return true;
        } else if (count - 1 >= min_count) {
            count = count - 1;
            return true;
        } else {
            return false;
        }
    }
}

static
bool Init(
    KDLRobotModel* model,
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    ROS_INFO("Initialize KDL Robot Model");
    if (!model->m_urdf.initString(robot_description)) {
        ROS_ERROR("Failed to parse the URDF.");
        return false;
    }

    ROS_INFO("Initialize Robot Model");
    urdf::JointSpec world_joint;
    world_joint.name = "map";
    world_joint.origin = Eigen::Affine3d::Identity(); // IMPORTANT
    world_joint.axis = Eigen::Vector3d::Zero();
    world_joint.type = urdf::JointType::Floating;
    if (!urdf::InitRobotModel(
            &model->m_robot_model, &model->m_urdf, &world_joint))
    {
        ROS_ERROR("Failed to initialize Robot Model");
        return false;
    }

    ROS_INFO("Initialize KDL tree");
    if (!kdl_parser::treeFromUrdfModel(model->m_urdf, model->m_tree)) {
        ROS_ERROR("Failed to parse the kdl tree from robot description.");
        return false;
    }

    ROS_INFO("Initialize KDL chain (%s, %s)", base_link.c_str(), tip_link.c_str());
    if (!model->m_tree.getChain(base_link, tip_link, model->m_chain)) {
        ROS_ERROR("Failed to fetch the KDL chain for the robot. (root: %s, tip: %s)", base_link.c_str(), tip_link.c_str());
        return false;
    }
    model->m_base_link = base_link;
    model->m_tip_link = tip_link;
    model->m_kinematics_link = GetLink(&model->m_robot_model, &base_link);
    if (model->m_kinematics_link == NULL) {
        return false; // this shouldn't happen if the chain initialized successfully
    }

    ROS_INFO("Gather joints in chain");
    std::vector<std::string> planning_joints;
    for (auto i = (unsigned)0; i < model->m_chain.getNrOfSegments(); ++i) {
        auto& segment = model->m_chain.getSegment(i);
        auto& child_joint_name = segment.getJoint().getName();
        auto* joint = GetJoint(&model->m_robot_model, &child_joint_name);
        if (GetVariableCount(joint) > 1) {
            ROS_WARN("> 1 variable per joint");
            return false;
        }
        if (GetVariableCount(joint) == 0) {
            continue;
        }
        planning_joints.push_back(child_joint_name);
    }

    ROS_INFO("Initialize URDF Robot Model with planning joints = %s", to_string(planning_joints).c_str());
    if (!urdf::Init(model, &model->m_robot_model, &planning_joints)) {
        ROS_ERROR("Failed to initialize URDF Robot Model");
        return false;
    }

    // do this after we've initialized the URDFRobotModel...
    model->planning_link = GetLink(&model->m_robot_model, &tip_link);
    if (model->planning_link == NULL) {
        return false; // this shouldn't happen either
    }

    // FK solver
    model->m_fk_solver = make_unique<KDL::ChainFkSolverPos_recursive>(model->m_chain);

    // IK Velocity solver
    model->m_ik_vel_solver = make_unique<KDL::ChainIkSolverVel_pinv>(model->m_chain);

    // IK solver
    KDL::JntArray q_min(model->jointVariableCount());
    KDL::JntArray q_max(model->jointVariableCount());
    for (size_t i = 0; i < model->jointVariableCount(); ++i) {
        if (model->vprops[i].continuous) {
            q_min(i) = -M_PI;
            q_max(i) = M_PI;
        } else {
            q_min(i) = model->vprops[i].min_position;
            q_max(i) = model->vprops[i].max_position;
        }
    }

    model->m_max_iterations = 200;
    model->m_kdl_eps = 0.001;
    model->m_ik_solver = make_unique<KDL::ChainIkSolverPos_NR_JL>(
            model->m_chain,
            q_min,
            q_max,
            *model->m_fk_solver,
            *model->m_ik_vel_solver,
            model->m_max_iterations,
            model->m_kdl_eps);

    model->m_jnt_pos_in.resize(model->m_chain.getNrOfJoints());
    model->m_jnt_pos_out.resize(model->m_chain.getNrOfJoints());
    model->m_free_angle = free_angle;
    model->m_search_discretization = 0.02;
    model->m_timeout = 0.005;
    return true;
}

bool KDLRobotModel::init(
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    return Init(this, robot_description, base_link, tip_link, free_angle);
}

auto KDLRobotModel::getBaseLink() const -> const std::string&
{
    return m_base_link;
}

auto KDLRobotModel::getPlanningLink() const -> const std::string&
{
    return m_tip_link;
}

static
void NormalizeAngles(KDLRobotModel* model, KDL::JntArray* q)
{
    for (auto i = 0; i < model->jointVariableCount(); ++i) {
        if (model->vprops[i].continuous) {
            (*q)(i) = sbpl::angles::normalize_angle((*q)(i));
        }
    }
}

static
double GetSolverMinPosition(KDLRobotModel* model, int vidx)
{
    if (model->vprops[vidx].continuous) {
        return -M_PI;
    } else {
        return model->vprops[vidx].min_position;
    }
}

bool KDLRobotModel::computeIKSearch(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution)
{
    // transform into kinematics and convert to kdl
    auto* T_map_kinematics = GetLinkTransform(&this->robot_state, m_kinematics_link);
    KDL::Frame frame_des;
    tf::transformEigenToKDL(T_map_kinematics->inverse() * pose, frame_des);

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        m_jnt_pos_in(i) = start[i];
    }

    // must be normalized for CartToJntSearch
    NormalizeAngles(this, &m_jnt_pos_in);

    auto initial_guess = m_jnt_pos_in(m_free_angle);

    auto start_time = sbpl::clock::now();
    auto loop_time = 0.0;
    auto count = 0;

    auto num_positive_increments =
            (int)((GetSolverMinPosition(this, m_free_angle) - initial_guess) /
                    this->m_search_discretization);
    auto num_negative_increments =
            (int)((initial_guess - GetSolverMinPosition(this, m_free_angle)) /
                    this->m_search_discretization);

    while (loop_time < this->m_timeout) {
        if (m_ik_solver->CartToJnt(m_jnt_pos_in, frame_des, m_jnt_pos_out) >= 0) {
            NormalizeAngles(this, &m_jnt_pos_out);
            solution.resize(start.size());
            for (size_t i = 0; i < solution.size(); ++i) {
                solution[i] = m_jnt_pos_out(i);
            }
            return true;
        }
        if (!getCount(count, num_positive_increments, -num_negative_increments)) {
            return false;
        }
        m_jnt_pos_in(m_free_angle) = initial_guess + this->m_search_discretization * count;
        ROS_DEBUG("%d, %f", count, m_jnt_pos_in(m_free_angle));
        loop_time = to_seconds(sbpl::clock::now() - start_time);
    }

    if (loop_time >= this->m_timeout) {
        ROS_DEBUG("IK Timed out in %f seconds", this->m_timeout);
        return false;
    } else {
        ROS_DEBUG("No IK solution was found");
        return false;
    }
    return false;
}

bool KDLRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    if (option != ik_option::UNRESTRICTED) {
        return false;
    }

    return computeIKSearch(pose, start, solution);
}

bool KDLRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option)
{
    // NOTE: only returns one solution
    RobotState solution;
    if (computeIK(pose, start, solution)) {
        solutions.push_back(solution);
    }
    return solutions.size() > 0;
}

bool KDLRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution)
{
    // transform into kinematics frame and convert to kdl
    auto* T_map_kinematics = GetLinkTransform(&this->robot_state, m_kinematics_link);
    KDL::Frame frame_des;
    tf::transformEigenToKDL(T_map_kinematics->inverse() * pose, frame_des);

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        m_jnt_pos_in(i) = start[i];
    }

    // must be normalized for CartToJntSearch
    NormalizeAngles(this, &m_jnt_pos_in);

    if (m_ik_solver->CartToJnt(m_jnt_pos_in, frame_des, m_jnt_pos_out) < 0) {
        return false;
    }

    NormalizeAngles(this, &m_jnt_pos_out);

    solution.resize(start.size());
    for (size_t i = 0; i < solution.size(); ++i) {
        solution[i] = m_jnt_pos_out(i);
    }

    return true;
}

void KDLRobotModel::printRobotModelInformation()
{
    leatherman::printKDLChain(m_chain, "robot_model");
}

auto KDLRobotModel::getExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<InverseKinematicsInterface>()) return this;
    return URDFRobotModel::getExtension(class_code);
}

} // namespace motion
} // namespace sbpl
