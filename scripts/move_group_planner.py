#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import franka_gripper.msg

import franka_control.srv

from math import pi
from moveit_commander.conversions import pose_to_list

GRIPPER_OPEN = [0.04, 0.4]
GRIPPER_CLOSED = [0.03, 0.3]

class MoveGroupPlanner():
    def __init__(self):

        ### MoveIt! 
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_planner',
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
        self.set_scene()
        print("============ Add object to planning scene")

        #self.gripper.set_joint_value_target(GRIPPER_OPEN)
        #self.gripper.go()


    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()
    def plan(self, goal):
        self.group.set_max_velocity_scaling_factor = 0.6
        self.group.set_max_acceleration_scaling_factor = 0.4
        
        self.group.set_start_state_to_current_state()
        trajectory = self.group.plan(goal)
        return trajectory

    def set_init(self, init):
        self.group.Init

    def set_scene(self):
        # self.scene.remove_world_object("glass")
        body = "/home/jiyeong/catkin_ws/src/assembly_with_object/mesh/body_in_m.stl"
        leg = "/home/jiyeong/catkin_ws/src/assembly_with_object/mesh/leg_in_m_2.stl"
        
        pose_ = geometry_msgs.msg.PoseStamped()
        pose_.header.frame_id = self.planning_frame #"panda_link0" #self.robot.get_planning_frame()
        pose_.pose.position.x = 0.3
        pose_.pose.position.y = -0.275
        pose_.pose.position.z = 0
        self.scene.add_mesh("lack_body", pose_, body, size = (1, 1, 1))
        
        
        rospy.sleep(1)

        pose_.pose.position.x = -0.35
        pose_.pose.position.y = -0.4
        pose_.pose.position.z = 0.4
        self.scene.add_mesh("leg1", pose_, leg, size = (1, 1, 1))

        #rospy.sleep(1)
        #pose_.pose.position.x = -0.35
        #pose_.pose.position.y = -0.6
        #pose_.pose.position.z = 0.4
        #self.scene.add_mesh("leg2", pose_, leg, size = (1, 1, 1))

        rospy.sleep(1)
        pose_.pose.position.x = -0.35
        pose_.pose.position.y = 0.4
        pose_.pose.position.z = 0.4
        self.scene.add_mesh("leg3", pose_, leg, size = (1, 1, 1))

        rospy.sleep(1)
        pose_.pose.position.x = -0.4
        pose_.pose.position.y = 0.6
        pose_.pose.position.z = 0.4
       # self.scene.add_mesh("leg4", pose_, leg, size = (1, 1, 1))
        #rospy.sleep(1)

    def plan_cartesian_target(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.18
        pose_goal.position.y = -0.38
        pose_goal.position.z = 0.52
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.group.plan(pose_goal)
        return trajectory
        #print (trajectory)
    
    def plan_cartesian_target_more(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.18
        pose_goal.position.y = -0.38
        pose_goal.position.z = 0.48
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.group.plan(pose_goal)
        return trajectory    

    def plan_cartesian_target2(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.35
        pose_goal.position.y = -0.26
        pose_goal.position.z = 0.53
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        self.group.set_planning_time(10)
        trajectory = self.group.plan(pose_goal)
        return trajectory
        #print (trajectory)
        

    def plan_cartesian_target3(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.18
        pose_goal.position.y = 0.43
        pose_goal.position.z = 0.52
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.group.plan(pose_goal)
        return trajectory
        #print (trajectory)

    def plan_cartesian_target3_more(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.18
        pose_goal.position.y = 0.43
        pose_goal.position.z = 0.48
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.group.plan(pose_goal)
        return trajectory

    def plan_cartesian_target4(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.9238795
        pose_goal.orientation.y = -0.3826834
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.    
        pose_goal.position.x = 0.35
        pose_goal.position.y = 0.245
        pose_goal.position.z = 0.53
        print(pose_goal)
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.group.plan(pose_goal)
        return trajectory
        

    def plan_joint_target(self):

        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2 
        joint_goal[4] = 0
        joint_goal[5] = pi/3 
        joint_goal[6] = pi/4
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.group.plan(joint_goal)
        return trajectory
        #print (trajectory)
        

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
    rospy.init_node('moveit_command_testS',
                    anonymous=True)
    mdp = MoveGroupPlanner()
    mdp.plan_cartesian_target()
    rospy.sleep(2)
    mdp.group.go()
    rospy.sleep(2)
    mdp.plan_cartesian_target_more()
    mdp.group.go()

    # mdp.gripper.set_joint_value_target(GRIPPER_CLOSED)
    # mdp.gripper.go()
    mdp.scene.attach_box(mdp.eef_link, "leg1")
    rospy.sleep(5)


    mdp.plan_cartesian_target2()
    rospy.sleep(2)
    mdp.group.go()

    # mdp.gripper.set_joint_value_target(GRIPPER_OPEN)
    # mdp.gripper.go()
    mdp.scene.remove_attached_object(mdp.eef_link, "leg1")
    rospy.sleep(5)

    mdp.plan_cartesian_target3()
    rospy.sleep(2)
    mdp.group.go()

    rospy.sleep(2)
    
    mdp.plan_cartesian_target3_more()
    mdp.group.go()
    
    # mdp.gripper.set_joint_value_target(GRIPPER_CLOSED)
    # mdp.gripper.go()
    mdp.scene.attach_box(mdp.eef_link, "leg3")
    rospy.sleep(5)

    mdp.plan_cartesian_target4()
    rospy.sleep(2)
    mdp.group.go()

    # mdp.gripper.set_joint_value_target(GRIPPER_OPEN)
    # mdp.gripper.go()

    mdp.scene.remove_attached_object(mdp.eef_link, "leg3")
    #mdp.gripper_open()

