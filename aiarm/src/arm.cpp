#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "arm.hpp"

Aiarm::arm::arm(char g_open,char gripper_ty,char arm_debug):g_open(g_open),gripper_ty(gripper_ty),arm_debug(arm_debug)
{
  static const std::string PLANNING_GROUP = "manipulator";

}




int main(int argc, char **argv)
{
  ros::init (argc, argv, "arm_move_group_tutorial");
}