cmake_minimum_required(VERSION 2.8.3)
project(assembly_with_object)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  std_srvs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  roscpp
  std_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

catkin_package()


add_definitions(-DDOF=7)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(include
${catkin_INCLUDE_DIRS}
#${EIGEN3_INCLUDE_DIRS}
#${RBDL_INCLDUE_DIR}
)

add_executable(stomp_planner 
    src/stomp_planner.cpp
)


target_link_libraries(stomp_planner
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)
