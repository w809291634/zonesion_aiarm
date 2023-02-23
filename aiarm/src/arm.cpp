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
#include <iostream>  

namespace  Aiarm{
  arm::arm(char g_open,char gripper_ty,char arm_debug):g_open(g_open),gripper_ty(gripper_ty),arm_debug(arm_debug)
  {
    static const std::string PLANNING_GROUP="manipulator";
    arm_move_group_.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
    arm_move_group_->setMaxVelocityScalingFactor(1.0f);            //设置允许的最大速度和加速度
    arm_move_group_->setMaxAccelerationScalingFactor(1.0f);
    /** \brief Set the tolerance that is used for reaching the goal. For
        joint state goals, this will be distance for each joint, in the
        configuration space (radians or meters depending on joint type). For pose
        goals this will be the radius of a sphere where the end-effector must
        reach. This function simply triggers calls to setGoalPositionTolerance(),
        setGoalOrientationTolerance() and setGoalJointTolerance(). */
    // arm_move_group_->setGoalTolerance(0.001)
    /** \brief Set the joint tolerance (for each joint) that is used for reaching the goal when moving to a joint value
     * target. */
    arm_move_group_->setGoalJointTolerance(0.001f);
    /** \brief Set the position tolerance that is used for reaching the goal when moving to a pose. */
    arm_move_group_->setGoalPositionTolerance(0.001f);
    /** \brief Set the orientation tolerance that is used for reaching the goal when moving to a pose. */
    arm_move_group_->setGoalOrientationTolerance(0.001f);

    arm_move_group_->setPlanningTime(5.0);
  }

}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "arm_move_group_tutorial");
  ros::NodeHandle node_handle;

  // 必须运行ROS旋转，MoveGroupInterface才能获取有关机器人状态的信息。这样做的一种方法是预先启动AsyncSpinner。
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Aiarm::arm test_arm(80);
  test_arm.all_gohome();
  while(1){
    printf("here\r\n");
    sleep(1);//延时1秒 
  }
}

