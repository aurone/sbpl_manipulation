////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush, Fahad Islam

#include <smpl/heuristic/workspace_dist_heuristic.h>

// standard includes
#include <cmath>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/graph/workspace_lattice_base.h>

namespace sbpl {
namespace motion {

static const char* LOG = "heuristic.workspace_dist";

static inline
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool WorkspaceDistHeuristic::init(RobotPlanningSpace* space)
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_point_ext = space->getExtension<PointProjectionExtension>();
    if (m_point_ext) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_pose_ext = space->getExtension<PoseProjectionExtension>();
    if (m_pose_ext) {
        SMPL_INFO_NAMED(LOG, "Got Pose Projection Extension!");
    }
    if (!m_pose_ext && !m_point_ext) {
        SMPL_WARN_NAMED(LOG, "WorkspaceDistHeuristic recommends PointProjectionExtension or PoseProjectionExtension");
    }

    m_ers = space->getExtension<ExtractRobotStateExtension>();
    m_rm_iface = space->robot()->getExtension<RedundantManipulatorInterface>();

    return true;
}

void WorkspaceDistHeuristic::setWeightX(double wx)
{
    m_x_coeff = wx;
}

void WorkspaceDistHeuristic::setWeightY(double wy)
{
    m_y_coeff = wy;
}

void WorkspaceDistHeuristic::setWeightZ(double wz)
{
    m_z_coeff = wz;
}

void WorkspaceDistHeuristic::setWeightRot(double wr)
{
    m_rot_coeff = wr;
}

void WorkspaceDistHeuristic::setWeightFreeAngles(double wfa)
{
    m_fa_coeff = wfa;
}

double WorkspaceDistHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    auto& goal_pose = planningSpace()->goal().pose;
    return EuclideanDistance(
            x, y, z,
            goal_pose.translation()[0],
            goal_pose.translation()[1],
            goal_pose.translation()[2]);
}

double WorkspaceDistHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: implement
    return 0.0;
}

Extension* WorkspaceDistHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int WorkspaceDistHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    if (m_rm_iface) {
        Eigen::Affine3d p;
        if (!m_pose_ext->projectToPose(state_id, p)) {
            return 0;
        }
        Eigen::Affine3d goal_pose;
        if (!m_pose_ext->projectToPose(planningSpace()->getGoalStateID(), goal_pose)) {
            return 0;
        }

        const double dist = computeDistance(p, goal_pose);

        // goal angles is also set as workspace state in Workspace Lattice Zero
        const WorkspaceState goal_state = planningSpace()->goal().angles;
        const WorkspaceState state = m_ers->extractState(state_id);
        
        Eigen::VectorXd fa_goal_state(m_rm_iface->redundantVariableCount());
        Eigen::VectorXd fa_state(m_rm_iface->redundantVariableCount());

        for (int i = 0; i < m_rm_iface->redundantVariableCount(); ++i) {
            fa_goal_state[i] = goal_state[6 + i];
            fa_state[i] = state[6 + i];
        }

        const double fa_dist = computeDistance(fa_goal_state, fa_state);
        const int h = FIXED_POINT_RATIO * (dist + fa_dist);

        double Y, P, R;
        angles::get_euler_zyx(p.rotation(), Y, P, R);
        SMPL_DEBUG_NAMED(LOG, "state_id %d, h(%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f) = %d", state_id, p.translation()[0], p.translation()[1], p.translation()[2], Y, P, R, h);
        return h;
    }
    else if (m_pose_ext) {
        Eigen::Affine3d p;
        if (!m_pose_ext->projectToPose(state_id, p)) {
            return 0;
        }

        auto& goal_pose = planningSpace()->goal().pose;

        const double dist = computeDistance(p, goal_pose);

        const int h = FIXED_POINT_RATIO * dist;

        double Y, P, R;
        angles::get_euler_zyx(p.rotation(), Y, P, R);
        SMPL_DEBUG_NAMED(LOG, "h(%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f) = %d", p.translation()[0], p.translation()[1], p.translation()[2], Y, P, R, h);

        return h;
    } else if (m_point_ext) {
        Eigen::Vector3d p;
        if (!m_point_ext->projectToPoint(state_id, p)) {
            return 0;
        }

        auto& goal_pose = planningSpace()->goal().pose;
        Eigen::Vector3d gp(goal_pose.translation());

        double dist = computeDistance(p, gp);

        const int h = FIXED_POINT_RATIO * dist;
        SMPL_DEBUG_NAMED(LOG, "h(%d) = %d", state_id, h);
        return h;
    } else {
        return 0;
    }
}

int WorkspaceDistHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int WorkspaceDistHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    // removed goal state cases (in euclid heuristic)
    if (m_rm_iface) {
        Eigen::Affine3d a, b;
        if (!m_pose_ext->projectToPose(from_id, a) ||
            !m_pose_ext->projectToPose(to_id, b))
        {
            return 0;
        }
        const double dist = computeDistance(a, b);

        const WorkspaceState& from_state = m_ers->extractState(from_id);
        const WorkspaceState& to_state = m_ers->extractState(to_id);

        Eigen::VectorXd fa_from_state(m_rm_iface->redundantVariableCount());
        Eigen::VectorXd fa_to_state(m_rm_iface->redundantVariableCount());

        for (int i = 0; i < m_rm_iface->redundantVariableCount(); ++i) {
            fa_from_state[i] = from_state[m_rm_iface->redundantVariableIndex(i)];
            fa_to_state[i] = to_state[m_rm_iface->redundantVariableIndex(i)];
        }
        const double fa_dist = computeDistance(fa_from_state, fa_to_state);
        const int h = FIXED_POINT_RATIO * (dist + fa_dist);

        return h;
    }
    else if (m_pose_ext) {
        if (from_id == planningSpace()->getGoalStateID()) {
            auto& gp = planningSpace()->goal().pose;
            Eigen::Affine3d p;
            if (!m_pose_ext->projectToPose(to_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(gp, p));
        } else if (to_id == planningSpace()->getGoalStateID()) {
            auto& gp = planningSpace()->goal().pose;
            Eigen::Affine3d p;
            if (!m_pose_ext->projectToPose(from_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(p, gp));
        } else {
            Eigen::Affine3d a, b;
            if (!m_pose_ext->projectToPose(from_id, a) ||
                !m_pose_ext->projectToPose(to_id, b))
            {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(a, b));
        }
    } else if (m_point_ext) {
        if (from_id == planningSpace()->getGoalStateID()) {
            Eigen::Vector3d gp(planningSpace()->goal().pose.translation());
            Eigen::Vector3d p;
            if (!m_pose_ext->projectToPoint(to_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(gp, p));
        } else if (to_id == planningSpace()->getGoalStateID()) {
            Eigen::Vector3d gp(planningSpace()->goal().pose.translation());
            Eigen::Vector3d p;
            if (!m_pose_ext->projectToPoint(from_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(p, gp));
        } else {
            Eigen::Vector3d a, b;
            if (!m_pose_ext->projectToPoint(from_id, a) ||
                !m_pose_ext->projectToPoint(to_id, b))
            {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(a, b));
        }
    } else {
        return 0;
    }
}

Eigen::Affine3d WorkspaceDistHeuristic::createPose(
    const std::vector<double> &pose) const
{
    return createPose(pose[0], pose[1], pose[2], pose[5], pose[4], pose[3]);
}

Eigen::Vector3d WorkspaceDistHeuristic::createPoint(
    const std::vector<double>& point) const
{
    return Eigen::Vector3d(point[0], point[1], point[2]);
}

Eigen::Affine3d WorkspaceDistHeuristic::createPose(
    double x, double y, double z,
    double Y, double P, double R) const
{
    return Eigen::Affine3d(
            Eigen::Translation3d(x, y, z) *
            Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX()));
}

double WorkspaceDistHeuristic::computeDistance(
    const Eigen::Affine3d& a,
    const Eigen::Affine3d& b) const
{
    auto sqrd = [](double d) { return d * d; };

    Eigen::Vector3d diff = b.translation() - a.translation();

    double dp2 =
            m_x_coeff * sqrd(diff.x()) +
            m_y_coeff * sqrd(diff.y()) +
            m_z_coeff * sqrd(diff.z());

    Eigen::Quaterniond qb(b.rotation());
    Eigen::Quaterniond qa(a.rotation());

    double dot = qa.dot(qb);
    if (dot < 0.0) {
        qb = Eigen::Quaterniond(-qb.w(), -qb.x(), -qb.y(), -qb.z());
        dot = qa.dot(qb);
    }

    double dr2 = angles::normalize_angle(2.0 * std::acos(dot));
    dr2 *= (m_rot_coeff * dr2);

    SMPL_DEBUG_NAMED(LOG, "Compute Distance: sqrt(%f + %f)", dp2, dr2);

    return std::sqrt(dp2 + dr2);
}

double WorkspaceDistHeuristic::computeDistance(
    const Eigen::Vector3d& u,
    const Eigen::Vector3d& v) const
{
    auto sqrd = [](double d) { return d * d; };
    Eigen::Vector3d diff = v - u;
    return m_x_coeff * sqrd(diff.x()) +
            m_y_coeff * sqrd(diff.y()) +
            m_z_coeff * sqrd(diff.z());
}

double WorkspaceDistHeuristic::computeDistance(
    const Eigen::VectorXd& u,
    const Eigen::VectorXd& v) const
{
    auto sqrd = [](double d) { return d * d; };
    Eigen::VectorXd diff = v - u;
    double dsum = 0.0;
    for (size_t i = 0; i < diff.size(); ++i) {
        dsum += sqrd(diff[i]);
    }
    return dsum;
}

} // namespace motion
} // namespace sbpl
