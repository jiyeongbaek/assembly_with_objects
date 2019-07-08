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

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].id = "box1";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.2;
  collision_objects[0].primitives[0].dimensions[2] = 1.0;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.30;
  collision_objects[0].primitive_poses[0].position.y = 0.40;
  collision_objects[0].primitive_poses[0].position.z = 0.25;
  collision_objects[0].primitive_poses[0].orientation.z = 1;

  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].header.frame_id = "panda_link0";
  collision_objects[1].id = "box2";
  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.1;
  collision_objects[1].primitives[0].dimensions[1] = 0.1;
  collision_objects[1].primitives[0].dimensions[2] = 1.0;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.30;
  collision_objects[1].primitive_poses[0].position.y = -0.30;
  collision_objects[1].primitive_poses[0].position.z = 0.14;
  collision_objects[1].primitive_poses[0].orientation.z = 1;

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
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
  
  // addCollisionObjects(planning_scene_interface);
  // ROS_INFO("Add objects to the scene");
  // sleep(3);

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
  move_group.setStartState(start_state);
  move_group.setJointValueTarget(final_state);

  // robot_state::RobotStatePtr robotstate;
  // robotstate=move_group.getCurrentState();
  // move_group.setStartState(*robotstate);

  //std::vector<double> jointvalue = { 0, -M_PI/4, 0, -M_PI/2, 0, M_PI/3, M_PI/4 };

  //plans_.push_back(getTrajectories(move_group, jointvalue, start_state)); //joint target
  MoveitPlan test_plan;
  bool success = (move_group.plan(test_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(test_plan);

  sleep(5);

  pose.position.x = 0.4;
  pose.position.y = -0.1;
  pose.position.z = 0.53;
  final_state.setFromIK(joint_model_group, pose);

  // moveit::core::robotStateToRobotStateMsg(current_state, start_state);
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setJointValueTarget(final_state);
  MoveitPlan test_plan2;
  move_group.plan(test_plan2);
  move_group.execute(test_plan2);

  sleep(5);

  pose.position.x = 0.;
  pose.position.y = 0.43;
  pose.position.z = 0.52;
  final_state.setFromIK(joint_model_group, pose);
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setJointValueTarget(final_state);
  MoveitPlan test_plan3;
  move_group.plan(test_plan3);
  move_group.execute(test_plan3);
        
 
  ros::shutdown();
  return 0;
}

