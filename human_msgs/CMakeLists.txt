cmake_minimum_required(VERSION 2.8.3)
project(human_msgs)

set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  nav_msgs
  tf
  visualization_msgs
)

## add message, serviece and action files

add_message_files( DIRECTORY msg
  FILES
    HumanPath.msg
    HumanPathArray.msg
    HumanTimeToGoal.msg
    HumanTimeToGoalArray.msg
    HumanTrajectory.msg
    HumanTrajectoryArray.msg
    HumanTwist.msg
    HumanTwistArray.msg
    TimeToGoal.msg
    TrackedSegment.msg
    TrackedHuman.msg
    TrackedHumanArray.msg
    TrackedHumans.msg
    TrackedSegmentType.msg
    Trajectory.msg
    TrajectoryArray.msg
    TrajectoryPoint.msg
    TrajectoryPointMsg.msg
    TrajectoryMsg.msg
    StateArray.msg
    HumanMarker.msg
    HumanMarkerStamped.msg
    HumanArray.msg
    TwistArray.msg
)

## generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

## declare catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_msgs
    nav_msgs
    tf
    visualization_msgs
)
