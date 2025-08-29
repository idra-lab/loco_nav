cmake_minimum_required(VERSION 3.0.2)
project(limo_description)

find_package(catkin REQUIRED COMPONENTS)



catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_install_python(
  PROGRAMS
    coontroller/spawn_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)



install(DIRECTORY controller/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/controller
  PATTERN ".svn" EXCLUDE)  
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE)



install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)



