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
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/ros.h>
#include <smpl/angles.h>
#include <eigen_conversions/eigen_kdl.h>

using namespace std;

namespace sbpl {
namespace motion {

static double normalizeAngle(double a, double a_min, double a_max)
{
    // normalize to [-2*pi, 2*pi] range
    if (std::fabs(a) > 2.0 * M_PI) {
        a = std::fmod(a, 2.0 * M_PI);
    }

    const double eps = 0.00001;
    if (fabs(a - a_max) > eps) {
        while (a > a_max) {
            a -= 2.0 * M_PI;
        }
    }
    if (fabs(a - a_min) > eps) {
        while (a < a_min) {
            a += 2.0 * M_PI;
        }
    }

    return a;
}

KDLRobotModel::KDLRobotModel() :
    initialized_(false),
    ik_solver_(),
    ik_vel_solver_(),
    fk_solver_()
{
}

KDLRobotModel::~KDLRobotModel()
{
}

bool KDLRobotModel::init(
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
    return true;
}

const std::string& KDLRobotModel::getKinematicsFrame() const
{
    return kinematics_frame_;
}

void KDLRobotModel::setKinematicsToPlanningTransform(
    const KDL::Frame& f,
    const std::string& name)
{
    T_kinematics_to_planning_ = f;
    T_planning_to_kinematics_ = f.Inverse();
}

void KDLRobotModel::normalizeAngles(KDL::JntArray& angles) const
{
    for (size_t i = 0; i < continuous_.size(); ++i) {
        if (continuous_[i]) {
            angles(i) = angles::normalize_angle(angles(i));
        }
    }
}

void KDLRobotModel::normalizeAngles(std::vector<double>& angles) const
{
    assert(continuous_.size() == angles.size());
    for (size_t i = 0; i < continuous_.size(); ++i) {
        if (continuous_[i]) {
            angles[i] = angles::normalize_angle(angles[i]);
        }
    }
}

bool KDLRobotModel::normalizeAnglesIntoRange(
    std::vector<double>& angles,
    const std::vector<double>& angle_mins,
    const std::vector<double>& angle_maxs) const
{
    size_t dim = angles.size();
    if (angle_mins.size() != dim || angle_maxs.size() != dim) {
        return false;
    }

    for (size_t i = 0; i < dim; i++) {
        if (angle_mins[i] > angle_maxs[i]) {
            return false;
        }
    }

    for (size_t i = 0; i < dim; i++) {
        double min_angle_norm = angles::normalize_angle(angle_mins[i]);
        angles[i] = normalizeAngle(angles[i], angle_mins[i], min_angle_norm);
        if (angles[i] < angle_mins[i] || angles[i] > angle_maxs[i]) {
            return false;
        }
    }

    return true;
}

bool KDLRobotModel::getJointLimits(
    std::vector<std::string>& joint_names,
    std::vector<double>& min_limits,
    std::vector<double>& max_limits,
    std::vector<bool>& continuous,
    std::vector<double>& vel_limits,
    std::vector<double>& eff_limits)
{
    min_limits.resize(joint_names.size());
    max_limits.resize(joint_names.size());
    continuous.resize(joint_names.size());
    vel_limits.resize(joint_names.size());
    eff_limits.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i].empty()) {
            ROS_ERROR("Empty joint name found.");
            return false;
        }
        bool c;
        if (!getJointLimits(
                joint_names[i],
                min_limits[i],
                max_limits[i],
                c,
                vel_limits[i],
                eff_limits[i]))
        {
            ROS_ERROR("Joint limits were not found for %s.", joint_names[i].c_str());
            return false;
        }
        continuous[i] = c;
    }
    return true;
}

bool KDLRobotModel::getJointLimits(
    std::string joint_name,
    double& min_limit,
    double& max_limit,
    bool& continuous,
    double& vel_limit,
    double& eff_limit)
{
    bool found_joint = false;
    auto link = urdf_.getLink(chain_tip_name_);
    while (link && (link->name != chain_root_name_) && !found_joint) {
        auto joint = urdf_.getJoint(link->parent_joint->name);
        if (joint->name.compare(joint_name) == 0) {
            if (joint->type != urdf::Joint::UNKNOWN &&
                joint->type != urdf::Joint::FIXED)
            {
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    continuous = false;

                    if (!joint->safety) {
                        min_limit = joint->limits->lower;
                        max_limit = joint->limits->upper;
                    } else {
                        min_limit = joint->safety->soft_lower_limit;
                        max_limit = joint->safety->soft_upper_limit;
                    }
                } else {
                    min_limit = -M_PI;
                    max_limit = M_PI;
                    continuous = true;
                }

                vel_limit = joint->limits->velocity;
                eff_limit = joint->limits->effort;
            }
            found_joint = true;
        }
        link = urdf_.getLink(link->getParent()->name);
    }
    return found_joint;
}

Extension* KDLRobotModel::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotModel>() ||
        class_code == GetClassCode<ForwardKinematicsInterface>() ||
        class_code == GetClassCode<InverseKinematicsInterface>())
    {
        return this;
    }

    return nullptr;
}

bool KDLRobotModel::checkJointLimits(
    const std::vector<double>& angles,
    bool verbose)
{
    std::vector<double> a = angles;
    if (!normalizeAnglesIntoRange(a, min_limits_, max_limits_)) {
        ROS_DEBUG("Joint angles are out of bounds.");
        return false;
    }

    return true;
}

bool KDLRobotModel::setPlanningLink(const std::string& name)
{
    planning_link_ = name;
    return true;
}

const std::string& KDLRobotModel::getPlanningLink() const
{
    return planning_link_;
}

bool KDLRobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    KDL::Frame& f)
{
    auto link_index_it = link_map_.find(name);
    if (link_index_it == end(link_map_)) {
        return false;
    }

    for (size_t i = 0; i < angles.size(); ++i) {
        jnt_pos_in_(i) = angles[i];
    }

    normalizeAngles(jnt_pos_in_);

    KDL::Frame f1;
    if (fk_solver_->JntToCart(jnt_pos_in_, f1, link_index_it->second) < 0) {
        ROS_ERROR("JntToCart returned < 0.");
        return false;
    }
    f = T_kinematics_to_planning_ * f1;

    return true;
}

Eigen::Affine3d KDLRobotModel::computeFK(const RobotState& angles)
{
    auto link_index_it = link_map_.find(planning_link_);
    if (link_index_it == end(link_map_)) {
        ROS_ERROR("Planning link is not part of the KDL chain");
        return Eigen::Affine3d::Identity();
    }

    KDL::Frame f, f1;
    for (size_t i = 0; i < angles.size(); ++i) {
        jnt_pos_in_(i) = angles[i];
    }
    normalizeAngles(jnt_pos_in_);

    if (fk_solver_->JntToCart(jnt_pos_in_, f1, link_index_it->second) < 0) {
        ROS_ERROR("JntToCart returned < 0.");
        return Eigen::Affine3d::Identity();
    }

    f = T_kinematics_to_planning_ * f1;

    Eigen::Affine3d out;
    tf::transformKDLToEigen(f, out);
    return out;
}

bool KDLRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    if (option == ik_option::RESTRICT_XYZ) {
        return false;
    }

    return computeIKSearch(pose, start, solution, 0.005);
}

bool KDLRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option)
{
    if (option == ik_option::RESTRICT_XYZ) {
        return false;
    }
    RobotState solution;
    if (computeIKSearch(pose, start, solution, 0.005)) {
        solutions.push_back(solution);
    }
    return solutions.size() > 0;
}

bool KDLRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution)
{
    KDL::Frame frame_des;
    tf::transformEigenToKDL(pose, frame_des);

    // transform into kinematics frame
    frame_des = T_planning_to_kinematics_ * frame_des;

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        jnt_pos_in_(i) = start[i];
    }

    // must be normalized for CartToJntSearch
    normalizeAngles(jnt_pos_in_);

    if (ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) < 0) {
        return false;
    }

    solution.resize(start.size());
    for (size_t i = 0; i < solution.size(); ++i) {
        solution[i] = jnt_pos_out_(i);
    }
    normalizeAngles(solution);

    return true;
}

bool KDLRobotModel::computeIKSearch(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution,
    double timeout)
{
    KDL::Frame frame_des;
    tf::transformEigenToKDL(pose, frame_des);

    // transform into kinematics frame
    frame_des = T_planning_to_kinematics_ * frame_des;

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        // must be normalized for CartToJntSearch
        jnt_pos_in_(i) = start[i];
    }
    normalizeAngles(jnt_pos_in_);

    double initial_guess = jnt_pos_in_(free_angle_);
    double search_discretization_angle = 0.02;

    ros::Time start_time = ros::Time::now();
    double loop_time = 0;
    int count = 0;

    int num_positive_increments = (int)((min_limits_[free_angle_] - initial_guess) / search_discretization_angle);
    int num_negative_increments = (int)((initial_guess - min_limits_[free_angle_]) / search_discretization_angle);
    while (loop_time < timeout) {
        if (ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) >= 0) {
            solution.resize(start.size());
            for (size_t i = 0; i < solution.size(); ++i) {
                solution[i] = jnt_pos_out_(i);
            }
            normalizeAngles(solution);
            return true;
        }
        if (!getCount(count, num_positive_increments, -num_negative_increments)) {
            return false;
        }
        jnt_pos_in_(free_angle_) = initial_guess + search_discretization_angle * count;
        ROS_DEBUG("%d, %f", count, jnt_pos_in_(free_angle_));
        loop_time = (ros::Time::now() - start_time).toSec();
    }
    if (loop_time >= timeout) {
        ROS_DEBUG("IK Timed out in %f seconds",timeout);
        return false;
    } else {
        ROS_DEBUG("No IK solution was found");
        return false;
    }
    return false;
}

void KDLRobotModel::printRobotModelInformation()
{
    leatherman::printKDLChain(kchain_, "robot_model");

    ROS_INFO("Joint<->Index Map:");
    for (auto iter = joint_map_.begin(); iter != joint_map_.end(); ++iter) {
        ROS_INFO("%22s: %d", iter->first.c_str(), iter->second);
    }

    ROS_INFO("Link<->KDL_Index Map:");
    for (auto iter = link_map_.begin(); iter != link_map_.end(); ++iter) {
        ROS_INFO("%22s: %d", iter->first.c_str(), iter->second);
    }
}

bool KDLRobotModel::getCount(
    int& count,
    const int& max_count,
    const int& min_count)
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

} // namespace motion
} // namespace sbpl
