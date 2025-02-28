cmake_minimum_required(VERSION 2.8.3)
project(hiperlab_rostools)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hiperlab_common
  hiperlab_components
  roscpp
  rospy
  message_generation
  std_msgs
  nav_msgs  
  tf
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

add_message_files(
  FILES
	estimator_output.msg  
	mocap_output.msg  
	simulator_truth.msg
	radio_command.msg
	telemetry.msg
	joystick_values.msg
  LoopPoseData.msg
  planner_diagnostics.msg
  planner_input.msg
	planner_output.msg
  planner_statistics.msg
	polynomial_trajectory.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
	std_msgs  # Or other packages containing msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hiperlab_rostools
  CATKIN_DEPENDS hiperlab_common hiperlab_components roscpp rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hiperlab_rostools.cpp
# )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hiperlab_rostools_node.cpp)
add_executable(quad_rappids_rates_control 
	src/QuadRappidsRatesControl/main.cpp
	src/QuadRappidsRatesControl/ExampleVehicleStateMachine.cpp
)
target_link_libraries(quad_rappids_rates_control ${catkin_LIBRARIES})
add_dependencies(quad_rappids_rates_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_vio_rates_control 
	src/QuadVIORatesControl/main.cpp
	src/QuadVIORatesControl/ExampleVehicleStateMachine.cpp
)
target_link_libraries(quad_vio_rates_control ${catkin_LIBRARIES})
add_dependencies(quad_vio_rates_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_mocap_rates_control 
	src/QuadMocapRatesControl/main.cpp
	src/QuadMocapRatesControl/ExampleVehicleStateMachine.cpp
)
target_link_libraries(quad_mocap_rates_control ${catkin_LIBRARIES})
add_dependencies(quad_mocap_rates_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_mocap_vio_init
	src/QuadMocapVIOInit/main.cpp
	src/QuadMocapVIOInit/ExampleVehicleStateMachine.cpp
)
target_link_libraries(quad_mocap_vio_init ${catkin_LIBRARIES})
add_dependencies(quad_mocap_vio_init ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_mocap_propeller_calibration src/QuadMocapPropellerCalibration/main_QuadMocapPropellerCalibration.cpp)
target_link_libraries(quad_mocap_propeller_calibration  ${catkin_LIBRARIES})
add_dependencies(quad_mocap_propeller_calibration   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_evaluator 
    src/QuadEvaluator/main_QuadEvaluator.cpp
    src/QuadEvaluator/QuadEvaluatorStateMachine.cpp
    )
target_link_libraries(quad_evaluator  ${catkin_LIBRARIES})
add_dependencies(quad_evaluator   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(demo_throw_and_recover 
	src/DEMO_ThrowAndRecover/main.cpp
	src/DEMO_ThrowAndRecover/VehicleStateMachine_demoThrow.cpp
	)
target_link_libraries(demo_throw_and_recover ${catkin_LIBRARIES})
add_dependencies(demo_throw_and_recover ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(demo_ultra_wideband 
    src/DEMO_UltraWideBand/main.cpp
    src/DEMO_UltraWideBand/VehicleStateMachine_UWBDemo.cpp
)
target_link_libraries(demo_ultra_wideband ${catkin_LIBRARIES})
add_dependencies(demo_ultra_wideband  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_mocap_acceleration_control src/QuadMocapAccelerationControl/main.cpp)
target_link_libraries(quad_mocap_acceleration_control ${catkin_LIBRARIES})
add_dependencies(quad_mocap_acceleration_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_mocap_tiltrotor_control src/QuadMocapTiltrotorControl/main.cpp)
target_link_libraries(quad_mocap_tiltrotor_control ${catkin_LIBRARIES})
add_dependencies(quad_mocap_tiltrotor_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_mocap_speeds_control src/QuadMocapSpeedsControl/main.cpp)
target_link_libraries(quad_mocap_speeds_control ${catkin_LIBRARIES})
add_dependencies(quad_mocap_speeds_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_manual_control src/QuadManualControl/main.cpp)
target_link_libraries(quad_manual_control ${catkin_LIBRARIES})
add_dependencies(quad_manual_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_manual_hybrid_control src/QuadManualHybridControl/main.cpp)
target_link_libraries(quad_manual_hybrid_control ${catkin_LIBRARIES})
add_dependencies(quad_manual_hybrid_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_outdoor_control src/QuadOutdoorControl/main.cpp)
target_link_libraries(quad_outdoor_control ${catkin_LIBRARIES})
add_dependencies(quad_outdoor_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simulator src/Simulator/main.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})
add_dependencies(simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sync_simulator src/SyncSimulator/main.cpp)
target_link_libraries(sync_simulator ${catkin_LIBRARIES})
add_dependencies(sync_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(testMotors src/TestMotors/main.cpp)
target_link_libraries(testMotors ${catkin_LIBRARIES})
add_dependencies(testMotors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(testMotorsIndividual src/TestMotorsIndividual/main.cpp)
# target_link_libraries(testMotorsIndividual ${catkin_LIBRARIES})
# add_dependencies(testMotorsIndividual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(testMotorsIndividualTimed src/TestMotorsIndividualTimed/main.cpp)
# target_link_libraries(testMotorsIndividualTimed ${catkin_LIBRARIES})
# add_dependencies(testMotorsIndividualTimed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(testMotorsTimed src/TestMotorsTimed/main.cpp)
# target_link_libraries(testMotorsTimed ${catkin_LIBRARIES})
# add_dependencies(testMotorsTimed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_joystick_acceleration_control src/JoystickAccelerationControl/main.cpp)
target_link_libraries(quad_joystick_acceleration_control ${catkin_LIBRARIES})
add_dependencies(quad_joystick_acceleration_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(vehicle_monitor 
    src/VehicleMonitor/main.cpp
    src/VehicleMonitor/VehicleMonitor.cpp
    src/VehicleMonitor/JoystickMonitor.cpp
)
target_link_libraries(vehicle_monitor ${catkin_LIBRARIES})
add_dependencies(vehicle_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mocap_to_rviz src/MocapToRviz/MocapToRviz_main.cpp)
target_link_libraries(mocap_to_rviz ${catkin_LIBRARIES})
add_dependencies(mocap_to_rviz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(PROGRAMS src/bagAutorecorder/bagAutorecorder.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hiperlab_rostools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
