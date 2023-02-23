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
  using namespace std;

  typedef std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupInterfacePtr;
  class arm
  {
    private:
      char g_open;        // 设置机械臂夹具
      char gripper_ty=1;
      char arm_debug=2;
      std::string PLANNING_GROUP = "manipulator";
      string current_pos="none";
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      // moveit::planning_interface::MoveGroupInterface arm_move_group{PLANNING_GROUP};    // 引用类作为数据方法1
      Aiarm::MoveGroupInterfacePtr arm_move_group_=nullptr; // 引用类作为数据方法2

    public:
      arm(char g_open,char gripper_ty=0,char arm_debug=0);
      ~arm(){};

    void all_gohome(){
      this->arm_move_group_->setNamedTarget("home");
      // bool success = (arm_move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");
      // arm_move_group_->execute(my_plan);
      this->arm_move_group_->move();
      this->current_pos="all_home";
    }

  };
  // arm::arm(/* args */);
  // arm::~arm();
}