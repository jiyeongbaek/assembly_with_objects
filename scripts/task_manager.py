#!/usr/bin/env python

from __future__ import print_function

import rospy
import moveit_commander
from smach import StateMachine
import smach_ros
import smach

from actionlib import *
from actionlib_msgs.msg import *
from smach_ros import SimpleActionState
from smach_ros import ServiceState
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

import franka_gripper.msg
import franka_control.srv

from string_transition_state import StringTransitionState
from move_group_planner import MoveGroupPlanner
from math import pi
from assembly_msgs.msg import AssemblePegInHoleAction, AssemblePegInHoleGoal
from assembly_msgs.msg import AssembleApproachAction, AssembleApproachGoal
import geometry_msgs

def main():
    rospy.init_node('assembly_task_manager')
    topic_name = '/assembly/state_transition'
    planner = MoveGroupPlanner()

    @smach.cb_interface(input_keys=['target_pose'])
    def get_trajectory(ud,goal):
        trajectory = planner.plan(ud.target_pose) # Good! Checked
        return_goal = FollowJointTrajectoryGoal()
        return_goal.trajectory = trajectory.joint_trajectory
        #print(trajectory)
        return return_goal

    assembly_sm = StateMachine(
            outcomes=['finished','aborted','preempted'])
    ## predefined targets
    # init position
    joint_init_goal = [None] * 7
    joint_init_goal[0] = 0
    joint_init_goal[1] = -pi/4
    joint_init_goal[2] = 0
    joint_init_goal[3] = -pi/2 
    joint_init_goal[4] = 0
    joint_init_goal[5] = pi/3 
    joint_init_goal[6] = pi/4


    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.9238795
    pose_goal.orientation.y = -0.3826834
    pose_goal.orientation.z = 0.
    pose_goal.orientation.w = 0.    
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.12

    
    assembly_sm.userdata.joint_init_goal = joint_init_goal
    assembly_sm.userdata.pose1_goal = pose_goal

    peg_goal1 = AssemblePegInHoleGoal()
    peg_goal1.pitch = 0.0025
    peg_goal1.linear_velocity = 0.005
    peg_goal1.z_force = -6.0
    peg_goal1.insert_force = -20.0

    contact_force_goal = AssembleApproachGoal()
    contact_force_goal.contact_force = 6

    epsilon_inner=0.002
    epsilon_outer=0.002 
    epsilon = franka_gripper.msg.GraspEpsilon(inner=epsilon_inner,
                                              outer=epsilon_outer)
    grasp_goal1 = franka_gripper.msg.GraspGoal(width=0.007,
                                            epsilon=epsilon,
                                            speed=0.1,
                                            force=60)

    gripper_open_goal = franka_gripper.msg.MoveGoal(width=0.1, speed=0.1)
    
    ## defining state machine structure
    with assembly_sm:
        StateMachine.add('READY',
            StringTransitionState(topic_name, outcomes=['initialize']), 
            transitions={'initialize':'INITIALIZE1'})

        StateMachine.add('INITIALIZE1',
            SimpleActionState('/franka_gripper/move', franka_gripper.msg.MoveAction, goal=gripper_open_goal),
            transitions={'succeeded':'INITIALIZE2'})

        StateMachine.add('INITIALIZE2',
            SimpleActionState('/assembly_controller/joint_trajectory_control', FollowJointTrajectoryAction, goal_cb=get_trajectory),
            transitions={'succeeded':'READY_TO_MOVE'}, remapping={'target_pose':'joint_init_goal'})
            
        StateMachine.add('READY_TO_MOVE',
            StringTransitionState(topic_name, outcomes=['approach1']), 
            transitions={'approach1':'APPROACH1'})

        StateMachine.add('APPROACH1',
            SimpleActionState('/assembly_controller/joint_trajectory_control', FollowJointTrajectoryAction, goal_cb=get_trajectory),
            transitions={'succeeded':'GRASP1'}, remapping={'target_pose':'pose1_goal'})

        StateMachine.add('GRASP1',
            SimpleActionState('/franka_gripper/grasp', franka_gripper.msg.GraspAction, goal=grasp_goal1),
            transitions={'succeeded':'APPROACH_TO_COMPONENT'})

        StateMachine.add('APPROACH_TO_COMPONENT',
            SimpleActionState('/assembly_controller/assemble_approach_control', AssembleApproachAction, goal=contact_force_goal),
            transitions={'succeeded':'ASSEMBLE1'})


        StateMachine.add('ASSEMBLE1',
            SimpleActionState('/assembly_controller/assemble_peginhole_control', AssemblePegInHoleAction, goal=peg_goal1),
            transitions={'succeeded':'finished'})



        #StateMachine.add('APPROACH1',
        #    SimpleActionState('/assembly_controller/joint_trajectory_control', JointControlAction, goal=joint_init_goal),
        #    transitions={'succeeded':'READY_TO_MOVE'})


    # Run state machine introspection server
    intro_server = smach_ros.IntrospectionServer('assembly',assembly_sm,'/ASSEMBLY_1')
    intro_server.start()
    assembly_sm.execute()

    rospy.spin()

    intro_server.stop()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
    
