// standard includes
#include <stdio.h>

// system includes
#include <ros/ros.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>
#include <urdf_parser/urdf_parser.h>

// project includes
#include <smpl_urdf_robot_model/smpl_urdf_robot_model.h>

namespace smpl = sbpl::motion;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_model_test");
    ros::NodeHandle nh;

    sbpl::VisualizerROS visualizer;
    sbpl::visual::set_visualizer(&visualizer);
    ros::Duration(1.0).sleep(); // give the publisher time to set up

    auto model = urdf::parseURDFFile(argv[1]);

    smpl::urdf::RobotModel robot_model;
    if (!InitRobotModel(&robot_model, model.get())) {
        return 1;
    }

    printf("Name: %s\n", GetName(&robot_model)->c_str());
    printf("Joint Count: %zu\n", GetJointCount(&robot_model));
    printf("Link Count: %zu\n", GetLinkCount(&robot_model));

    printf("Links:\n");
    for (auto& link : Links(&robot_model)) {
        printf("  %s\n", link.name.c_str());
        printf("    parent joint: %s\n", link.parent ? link.parent->name.c_str() : "(none)");
        printf("    child joints:\n");
        for (auto* child = link.children; child != NULL; child = child->sibling) {
            printf("      %s\n", child->name.c_str());
        }
    }

    printf("Joints:\n");
    for (auto& joint : Joints(&robot_model)) {
        printf("  %s\n", joint.name.c_str());
    }

    printf("Variables:\n");
    for (auto& variable : Variables(&robot_model)) {
        printf("  %s\n", variable.name.c_str());
        printf("    has position limits: %s\n", variable.limits.has_position_limits ? "true" : "false");
        if (variable.limits.has_position_limits) {
            printf("    min_position: %f\n", variable.limits.min_position);
            printf("    max_position: %f\n", variable.limits.max_position);
            printf("    max_velocity: %f\n", variable.limits.max_velocity);
            printf("    max_effort: %f\n", variable.limits.max_effort);
        }
        printf("    joint: %s\n", variable.joint->name.c_str());
    }

    smpl::urdf::RobotState robot_state;
    Init(&robot_state, &robot_model);

    SetToDefaultValues(&robot_state);
    printf("default robot_state:\n");
    for (auto& variable : Variables(&robot_model)) {
        printf("%f\n", GetVariablePosition(&robot_state, &variable));
    }

    printf("transforms:\n");
    UpdateTransforms(&robot_state);
    for (int i = 0; i < GetLinkCount(&robot_model); ++i) {
        auto* trans = GetLinkTransform(&robot_state, i);
        printf("  %s: [ [ %f, %f, %f, %f ], [ %f, %f, %f, %f ], [ %f, %f, %f, %f ], [ %f, %f, %f, %f ] ]\n",
                GetLinkName(&robot_model, i)->c_str(),
                (*trans)(0, 0), (*trans)(1, 0), (*trans)(2, 0), (*trans)(3, 0),
                (*trans)(0, 1), (*trans)(1, 1), (*trans)(2, 1), (*trans)(3, 1),
                (*trans)(0, 2), (*trans)(1, 2), (*trans)(2, 2), (*trans)(3, 2),
                (*trans)(0, 3), (*trans)(1, 3), (*trans)(2, 3), (*trans)(3, 3));

    }

    auto gray = sbpl::visual::Color{ 0.5f, 0.5f, 0.5f, 1.0f };
    SV_SHOW_INFO(MakeRobotVisualization(&robot_state, gray, "map", "test_visual"));
    SV_SHOW_INFO(MakeCollisionVisualization(&robot_state, gray, "map", "test_collision"));

    ros::Duration(1.0).sleep(); // give the publisher time to visualize

    return 0;
}

