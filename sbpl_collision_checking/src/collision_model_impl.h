////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef sbpl_collision_CollisionModelImpl_h
#define sbpl_collision_CollisionModelImpl_h

#include <map>
#include <string>
#include <vector>

#include <kdl/kdl.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/MultiDOFJointState.h>

#include <sbpl_collision_checking/group.h>

namespace sbpl {
namespace collision {

struct CollisionModelConfig;

class CollisionModelImpl
{
public:

    CollisionModelImpl();
    ~CollisionModelImpl();

    bool init(
        const std::string& urdf_string,
        const CollisionModelConfig& config);

    bool setDefaultGroup(const std::string& group_name);

    void setOrderOfJointPositions(
        const std::vector<std::string>& joint_names,
        const std::string& group_name);

    std::string getReferenceFrame(const std::string& group_name) const;

    void getGroupNames(std::vector<std::string>& names) const;

    bool getJointLimits(
        const std::string& group_name,
        const std::string& joint_name,
        double& min_limit,
        double& max_limit,
        bool& continuous) const;

    bool getFrameInfo(
        const std::string& name,
        const std::string& group_name,
        int& chain,
        int& segment) const;

    bool doesLinkExist(const std::string& name, const std::string& group_name) const;

   const std::vector<const Sphere*>& getDefaultGroupSpheres() const;

    bool setWorldToModelTransform(
        const moveit_msgs::RobotState& state,
        const std::string& world_frame);

    void setJointPosition(const std::string& name, double position);

    Group* getGroup(const std::string& name);

    void getVoxelGroups(std::vector<Group*>& vg);

    bool computeDefaultGroupFK(
        const std::vector<double>& angles,
        std::vector<std::vector<KDL::Frame>>& frames);

    bool computeGroupFK(
            const std::vector<double>& angles,
            Group* group,
            std::vector<std::vector<KDL::Frame>>& frames);

    void printGroups() const;

    void printDebugInfo(const std::string& group_name) const;

private:

    struct KDLJointMapping;
    {
        std::vector<int> chain_indices;
        std::vector<int> joint_indices;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    boost::shared_ptr<urdf::Model> urdf_;

    std::string m_model_frame;
    KDL::Tree m_tree;
    // chains leading up to root links of groups
    std::vector<KDL::Chain> m_chains;
    std::vector<std::string> m_chain_tips;
    std::vector<KDL::ChainFkSolverPos_recursive> m_solvers;
    std::vector<KDL::JntArray> m_joint_arrays;
    std::map<std::string, KDLJointMapping> m_joint_map;

    robot_model_loader::RobotModelLoaderPtr rm_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    std::map<std::string, Group*> group_config_map_;
    Group* dgroup_;

    bool initURDF(const std::string &urdf_string);
    bool initKdlRobotModel();
    bool initMoveItRobotModel(const std::string &urdf_string);
    bool initAllGroups(const CollisionModelConfig& config);

    void decomposeRobotModel();
};

} // namespace collision
} // namespace sbpl

#endif
