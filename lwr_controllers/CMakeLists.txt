cmake_minimum_required(VERSION 2.8.3)
project(lwr_controllers)

# Load catkin and all dependencies required for this package
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_message_files(
   FILES
   PoseRPY.msg
   RPY.msg
   MultiPriorityTask.msg
   PID.msg
   PIDgains.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


# Declare catkin package
catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    kdl_parser
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/joint_impedance_controller.cpp	
  src/inverse_dynamics_controller.cpp
  src/computed_torque_controller.cpp
  src/one_task_inverse_kinematics.cpp
  src/multi_task_priority_inverse_kinematics.cpp
  src/multi_task_priority_inverse_dynamics.cpp
  src/minimum_effort_inverse_dynamics.cpp
  src/backstepping_controller.cpp
  src/dynamic_sliding_mode_controller.cpp
  src/dynamic_sliding_mode_controller_task_space.cpp
  src/one_task_inverse_dynamics_jl.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES lwr_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})