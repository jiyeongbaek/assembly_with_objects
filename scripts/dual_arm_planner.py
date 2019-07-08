#!/usr/bin/env python
<<<<<<< HEAD
# -*- coding: utf-8 -*-
=======
>>>>>>> 0e5020a4f1c82d78d18c07d4c27f05eb1828d791

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
<<<<<<< HEAD

import franka_gripper.msg
=======
import franka_gripper.msg

>>>>>>> 0e5020a4f1c82d78d18c07d4c27f05eb1828d791
import franka_control.srv

from math import pi
from moveit_commander.conversions import pose_to_list

<<<<<<< HEAD

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info")

=======
>>>>>>> 0e5020a4f1c82d78d18c07d4c27f05eb1828d791
GRIPPER_OPEN = [0.04, 0.4]
GRIPPER_CLOSED = [0.03, 0.3]

class MoveGroupPlanner():
    def __init__(self):

        ### MoveIt! 
        moveit_commander.roscpp_initialize(sys.argv)
<<<<<<< HEAD
        rospy.init_node('move_group_planner',
                        anonymous=True)
=======
        #rospy.init_node('move_group_planner',
        #                anonymous=True)
>>>>>>> 0e5020a4f1c82d78d18c07d4c27f05eb1828d791

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.left_group_name = "left_arm"
        self.left_group = moveit_commander.MoveGroupCommander(self.left_group_name)
        self.right_group_name = "right_arm"
        self.right_group = moveit_commander.MoveGroupCommander(self.right_group_name)
        # self.gripper = moveit_commander.MoveGroupCommander("hand")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
       
        # We can also print the name of the end-effector link for this group:
        self.left_eef_link = self.left_group.get_end_effector_link()
        self.right_eef_link = self.right_group.get_end_effector_link()
        
        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        self.box_name = ''
        self.active_controllers = []

        rospy.sleep(1)
        #self.set_scene()
        pose_ = geometry_msgs.msg.PoseStamped()
        pose_.header.frame_id = self.robot.get_planning_frame()
        pose_.pose.position.x = 0.
        pose_.pose.position.y = 0.5
        pose_.pose.position.z = 1.2
        self.scene.add_box("box1", pose_, size=(1, 1, 3))
        print("============ Add object to planning scene")

        #self.gripper.set_joint_value_target(GRIPPER_OPEN)
        #self.gripper.go()


    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()
    # def plan(self, goal):
    #     self.group.set_max_velocity_scaling_factor = 0.6
    #     self.group.set_max_acceleration_scaling_factor = 0.4
        
    #     self.group.set_start_state_to_current_state()
    #     trajectory = self.group.plan(goal)
    #     return trajectory

    # def set_init(self, init):
    #     self.group.Init

    def set_scene(self):
        self.scene.remove_world_object("leg1")
        leg = "/home/jiyeong/catkin_ws/src/assembly_with_object/mesh/leg_in_m_2.dae"
        
        pose_ = geometry_msgs.msg.PoseStamped()
        pose_.header.frame_id = self.robot.get_planning_frame()
        pose_.pose.position.x = 0.
        pose_.pose.position.y = 0.5
        pose_.pose.position.z = 1.2
        self.scene.add_mesh("leg1", pose_, leg, size = (1, 1, 1))

    def plan_cartesian_target(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.5
        pose_goal.position.z = 1.32
        print(pose_goal)
        trajectory = self.left_group.plan(pose_goal)
        return trajectory
        #print (trajectory)

    def plan_cartesian_target2(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0
        pose_goal.position.z = 1.32
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.left_group.plan(pose_goal)
        return trajectory
        #print (trajectory)
        

    def plan_cartesian_target3(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0
        pose_goal.position.z = 1.32
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.right_group.plan(pose_goal)
        return trajectory
        #print (trajectory)

  
    def plan_cartesian_target4(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.5
        pose_goal.position.y = -0.5
        pose_goal.position.z = 1.32
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.right_group.plan(pose_goal)
        return trajectory
            

    def display_trajectory(self, plan):
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(self.display_trajectory)


    # def grasp(self, width,
    #                 epsilon_inner=0.005, epsilon_outer=0.005,
    #                 speed=0.1, force=35):
    #     epsilon = franka_gripper.msg.GraspEpsilon(inner=epsilon_inner,
    #                                             outer=epsilon_outer)
    #     goal = franka_gripper.msg.GraspGoal(width=width,
    #                                         epsilon=epsilon,
    #                                         speed=speed,
    #                                         force=force)
    #     rospy.loginfo('Grasping:\n{}'.format(goal))
    #     self.gripper_grasp.send_goal(goal)
    #     self.gripper_grasp.wait_for_result()

    # def gripper_open(self, width=0.1,
    #                 epsilon_inner=0.005, epsilon_outer=0.005,
    #                 speed=0.1, force=10):
    #     goal = franka_gripper.msg.MoveGoal(width=width, speed=speed)
    #     rospy.loginfo('Moving gripper:\n{}'.format(goal))
    #     self.gripper_move.send_goal(goal)
    #     self.gripper_move.wait_for_result()
    #     if not self.gripper_move.get_result().success:
    #         rospy.logerr("Couldn't move gripper")
    #         sys.exit(1)

if __name__ == '__main__':
    #rospy.init_node('moveit_command_test',
    #                anonymous=True)
    mdp = MoveGroupPlanner()

    #left arm move
    mdp.plan_cartesian_target()
    rospy.sleep(2)
    mdp.left_group.go()
    rospy.sleep(2)
    #mdp.scene.attach_box(mdp.left_eef_link, "leg1")
    #rospy.sleep(5)

    mdp.plan_cartesian_target2()
    rospy.sleep(2)
    mdp.left_group.go()
    #mdp.scene.remove_attached_object(mdp.left_eef_link, "leg1")

    mdp.plan_cartesian_target()
    rospy.sleep(2)
    mdp.left_group.go()
    rospy.sleep(2)

    ## right arm move
    mdp.plan_cartesian_target3()
    rospy.sleep(2)
    mdp.right_group.go()
    rospy.sleep(2)
    #mdp.scene.attach_box(mdp.right_eef_link, "leg1")
    #rospy.sleep(5)

    mdp.plan_cartesian_target4()
    rospy.sleep(2)
    mdp.right_group.go()
    #mdp.scene.remove_attached_object(mdp.right_eef_link, "leg1")

    # # mdp.gripper.set_joint_value_target(GRIPPER_OPEN)
    # # mdp.gripper.go()
    # mdp.scene.remove_attached_object(mdp.eef_link, "leg1")
    # rospy.sleep(5)


