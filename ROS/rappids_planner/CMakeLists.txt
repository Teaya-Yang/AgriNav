cmake_minimum_required(VERSION 2.8.3)
project(rappids_planner)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  tf
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  drone_status.msg
  planner_diagnostics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
	geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(rappids_library STATIC
        src/depth_image_planner.cpp
        src/quartic.cpp
        src/rapid_trajectory_generator.cpp
        src/single_axis_trajectory.cpp)

add_executable(rappids_perception_aware
  src/perception_aware/main.cpp
  src/perception_aware/rappids_perception_aware.cpp
)
target_link_libraries(rappids_perception_aware
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rappids_library
)
add_dependencies(rappids_perception_aware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(single_image_test
  src/single_image_test/single_image_test.cpp)
target_link_libraries(single_image_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rappids_library
)
add_dependencies(single_image_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(feature_publisher
  src/features_publisher/main.cpp)
target_link_libraries(feature_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(feature_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rtabmap_odom
  src/rtabmap_odom/main.cpp)
target_link_libraries(rtabmap_odom
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(rtabmap_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(my_tf_broadcaster src/tf_publisher/main.cpp)
target_link_libraries(my_tf_broadcaster ${catkin_LIBRARIES})