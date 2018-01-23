cmake_minimum_required(VERSION 2.8.3)
project(numl_val_motion_tutorials)

find_package(catkin REQUIRED COMPONENTS
  ihmc_msgs
  rospy
  roscpp
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_arms src/control_arms.cpp)
target_link_libraries(control_arms ${catkin_LIBRARIES})

add_executable(control_chest_orientation src/control_chest_orientation.cpp)
target_link_libraries(control_chest_orientation ${catkin_LIBRARIES})

add_executable(control_neck src/control_neck.cpp)
target_link_libraries(control_neck ${catkin_LIBRARIES})

add_executable(control_pelvis_orientation src/control_pelvis_orientation.cpp)
target_link_libraries(control_pelvis_orientation ${catkin_LIBRARIES})


