#!/usr/bin/env python
from __future__ import print_function

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time
import sys
import copy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg

class MoveGroupPlanner():
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        #                anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        # self.gripper = moveit_commander.MoveGroupCommander("hand")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        self.box_name = ''
        self.active_controllers = []

        rospy.sleep(1)

    def plan1(self):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.18 #0.4
        pose_goal.position.y = -0.38 #0.0
        pose_goal.position.z = 0.52 #0.12
        print(pose_goal)
          
        self.group.set_joint_value_target(pose_goal)
        rospy.sleep(3)
        
    def plan2(self):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.4
        pose_goal.position.y = -0.1
        pose_goal.position.z = 0.53
        print(pose_goal)
       
        self.group.set_joint_value_target(pose_goal)
        rospy.sleep(3)
        

    def plan3(self):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.0
        pose_goal.position.y = 0.43
        pose_goal.position.z = 0.52
        # pose_goal.position.x = 0.5
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.12
        print(pose_goal)
        self.group.set_joint_value_target(pose_goal)
        rospy.sleep(3)
        

if __name__ == "__main__":
    rospy.init_node('moveit_command_testS',
                    anonymous=True)
    mdp = MoveGroupPlanner()
    mdp.plan1()
    #rospy.sleep(2)
    mdp.group.go()

    rospy.sleep(2)
    mdp.plan2()
    #rospy.sleep(2)
    mdp.group.go()

    rospy.sleep(2)
    mdp.plan3()
    #rospy.sleep(2)
    mdp.group.go()
    
    