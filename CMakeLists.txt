cmake_minimum_required(VERSION 2.8.3)
project(assembly_with_object)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  std_srvs
)

catkin_package()
