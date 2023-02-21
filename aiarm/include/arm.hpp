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


namespace Aiarm{
  class arm
  {
    private:
      char g_open;        // 设置机械臂夹具
      char gripper_ty;
      char arm_debug;
      std::string PLANNING_GROUP = "manipulator";
      moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

    public:
      arm(char g_open,char gripper_ty=0,char arm_debug=0);
      ~arm();
  };
  // arm::arm(/* args */);
  // arm::~arm();
}