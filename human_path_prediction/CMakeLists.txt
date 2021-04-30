
cmake_minimum_required(VERSION 2.8.3)
project(human_path_prediction)

set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  human_msgs
  message_generation
  nav_msgs
  roscpp
  std_srvs
  tf
  visualization_msgs
)

## install python modules and global scripts
# catkin_python_setup()

# add message, serviece and action files

add_message_files( DIRECTORY msg
  FILES
    PredictedPoses.msg
    PredictedGoal.msg
    HumanPose.msg
)

add_service_files( DIRECTORY srv
  FILES
    HumanPosePredict.srv
    HumanGoal.srv
)

# add_action_files( DIRECTORY action
#   FILES
#     Action1.action
# )

# generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    human_msgs
)

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/HumanPathPrediction.cfg
)

## declare catkin package
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES human_path_prediction
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    human_msgs
    message_runtime
    nav_msgs
    roscpp
    std_srvs
    tf
    visualization_msgs
#   DEPENDS
#     system_lib
)



## build ##

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# declare a c++ executable
add_executable(human_path_prediction
  src/human_path_prediction.cpp
)

# cmake target dependencies of the c++ executable
add_dependencies(human_path_prediction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ executable against
target_link_libraries(human_path_prediction ${catkin_LIBRARIES})



## install ##

## executable scripts (python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## executables and/or libraries for installation
# install(TARGETS human_path_prediction human_path_prediction_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## cpp-header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## other files for installation (launch, data, bag files)
# foreach(dir launch data bags)
#    install(DIRECTORY ${dir}/
#       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

## other files for installation
# install(
#   FILES
#     myfile1
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



## testing ##

## add catch based cpp test target
# catkin_add_catchtest(${PROJECT_NAME}-test test/test_hanp_prediction.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## add nosetest file folders
# catkin_add_nosetests(test)

## add helpers for testing
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)
#add_executable(prediction_service_caller
#  test/prediction_service_caller.cpp
#)
#add_dependencies(prediction_service_caller
#  ${PROJECT_NAME}_generate_messages_cpp
#)
#target_link_libraries(prediction_service_caller
#  ${catkin_LIBRARIES}
#)
