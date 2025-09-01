cmake_minimum_required(VERSION 3.0.2)
project(limo_description)

find_package(catkin REQUIRED std_msgs message_generation COMPONENTS)



add_message_files(
  DIRECTORY
  msg
  FILES
  Reference.msg
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  CATKIN_DEPENDS rospy message_generation std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/lyapunov.py
    scripts/math_tools.py
    scripts/utils.py
    scripts/params.py
    scripts/trajectory.py
    scripts/unicycle.py
    scripts/velocity_generator.py
    scripts/controller.py
    scripts/spawn_controllers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
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



