cmake_minimum_required(VERSION 3.0.2)
project(map_pkg)

## Compile settings
add_compile_options(-Wall -Wextra -Wpedantic)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_ros
  message_generation
  obstacles_msgs
  
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs  obstacles_msgs 
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Python scripts (optional install)
catkin_install_python(PROGRAMS
  scripts/spawn_obstacles.py
  scripts/spawn_gates.py
  scripts/spawn_borders.py
  scripts/spawn_utility_.py
  scripts/spawn_borders.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## C++ executables
add_executable(send_obstacles src/send_obstacles.cpp src/utilities.cpp)
add_executable(send_gates src/send_gates.cpp src/utilities.cpp)
add_executable(send_victims src/send_victims.cpp src/utilities.cpp)
add_executable(send_borders src/send_borders.cpp src/utilities.cpp)
add_executable(send_timeout src/send_timeout.cpp src/utilities.cpp)

target_link_libraries(send_obstacles ${catkin_LIBRARIES})
target_link_libraries(send_gates ${catkin_LIBRARIES})
target_link_libraries(send_victims ${catkin_LIBRARIES})
target_link_libraries(send_borders ${catkin_LIBRARIES})
target_link_libraries(send_timeout ${catkin_LIBRARIES})

install(TARGETS send_obstacles send_gates send_victims send_borders send_timeout
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
  
 install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)
  
   
 install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  PATTERN ".svn" EXCLUDE)
  
  
    
   
 install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)
  
  
    
 install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)
  
  install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE)
  
 install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
  PATTERN ".svn" EXCLUDE)
  
  
catkin_install_python(
  PROGRAMS
    scripts/spawn_obstacles.py
    scripts/spawn_gates.py
	scripts/create_map_pgm.py
	scripts/generate_config_file.py
	scripts/ground_truth_odom.py
	scripts/geo_utility.py
    scripts/spawn_borders.py
    scripts/spawn_utility_.py
    scripts/spawn_borders.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  
