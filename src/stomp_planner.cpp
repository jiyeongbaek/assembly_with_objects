#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>

//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

static const std::string PLANNING_GROUP = "panda_arm";
typedef moveit::planning_interface::MoveGroupInterface::Plan MoveitPlan;
std::vector<MoveitPlan> plans_; 
std::vector<geometry_msgs::Pose> target_poses_;

MoveitPlan getTrajectories
(moveit::planning_interface::MoveGroupInterface& group, geometry_msgs::Pose target_pose, moveit_msgs::RobotState& start_state)
{  
  MoveitPlan plan_;
  group.setStartState(start_state);
  group.setPoseTarget(target_pose);
  
  group.plan(plan_); // the resulting plan is stored in 'plan'
  for (int i = 0; i < 7 ; i ++){
    start_state.joint_state.position[i] = plan_.trajectory_.joint_trajectory.points.back().positions[i];
  }
  return plan_;
}


MoveitPlan getTrajectories
(moveit::planning_interface::MoveGroupInterface& group, std::vector<double> joint_values, moveit_msgs::RobotState& start_state)
  {  
  MoveitPlan plan_;
  group.setStartState(start_state);
  group.setJointValueTarget(joint_values);

  group.plan(plan_); // the resulting plan is stored in 'plan'
  for (int i = 0; i < 7 ; i ++){
    start_state.joint_state.position[i] = plan_.trajectory_.joint_trajectory.points.back().positions[i];
  }
  return plan_;
}

  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_motion_plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("platform_base");

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  geometry_msgs::Pose pose; 
  pose.orientation.x = 0.9238795;
  pose.orientation.y = -0.3826834;
  pose.orientation.z = 0;
  pose.orientation.w = 0;    
  pose.position.x = 0.18;
  pose.position.y = -0.38;
  pose.position.z = 0.48;

  
  moveit_msgs::RobotState start_state;
  const moveit::core::RobotState current_state = *move_group.getCurrentState();
  moveit::core::robotStateToRobotStateMsg(current_state, start_state);
  
  robot_state::RobotState final_state(*move_group.getCurrentState());
  final_state.setFromIK(joint_model_group, pose);
  move_group.setJointValueTarget(final_state);

  //std::vector<double> jointvalue = { 0, -M_PI/4, 0, -M_PI/2, 0, M_PI/3, M_PI/4 };

  //plans_.push_back(getTrajectories(move_group, jointvalue, start_state)); //joint target
  MoveitPlan test_plan;
  bool success = (move_group.plan(test_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

 
  ros::shutdown();
  return 0;
}

