#include <iostream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  moveit_msgs::CollisionObject beer_bottle;
  shapes::Mesh *bottle_mesh = shapes::createMeshFromResource("package://add_object_stl/meshes/beer_bottle_up_75.stl");
//  beer_bottle.link_name = "panda_link0";
  beer_bottle.header.frame_id = "panda_link0";
  beer_bottle.id = "beer_bottle";

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(bottle_mesh, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  beer_bottle.mesh_poses.resize(1);
  beer_bottle.mesh_poses[0].position.x = 0.75; //0.7
  beer_bottle.mesh_poses[0].position.y = 0;
  beer_bottle.mesh_poses[0].position.z = 0.5;
  beer_bottle.mesh_poses[0].orientation.w= 0.0; 
  beer_bottle.mesh_poses[0].orientation.x= 0.0; 
  beer_bottle.mesh_poses[0].orientation.y= 0.0;
  beer_bottle.mesh_poses[0].orientation.z= 0.0;
  beer_bottle.meshes.push_back(mesh);	
  beer_bottle.operation=beer_bottle.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(beer_bottle);
  planning_scene_interface.addCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
 
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  
  addCollisionObjects(planning_scene_interface);
  ROS_INFO("Add object to planning scene");

  std::vector<geometry_msgs::Pose> target_poses_;
  geometry_msgs::Pose pose; 
  pose.position.x = 0.6;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  target_poses_.push_back(pose);

  pose.position.x = 0;
  pose.position.y = 0.5;
  pose.position.z = 0.5;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI / 2);
  target_poses_.push_back(pose);

  group.setPoseTarget(target_poses_[0]);
  //cout << "target pose : "<< target_poses_[0].position.x << ", " << target_poses_[0].position.y << ", " << target_poses_[0].position.z << endl;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  bool success = (group.plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("test", " Plan 1 is %s", success ? "SUCCESS":"FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to EXECUTE");
  //group.execute(plan_);
  group.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to ATTACH object to robot");
  // Attach object to robot
  group.attachObject("beer_bottle");
  cout <<  "Attach the object to the robot" << endl;
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan target2");

  group.setPoseTarget(target_poses_[1]);
  moveit::planning_interface::MoveGroupInterface::Plan plan1_;

  bool success2 = (group.plan(plan1_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("test", "Visualizing plan 2 (pose goal) %s", success2 ? "":"FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to EXECUTE");
  //group.execute(plan1_); 
  group.move();
  group.detachObject("beer_bottle");
  ros::waitForShutdown();
  return 0;
}
