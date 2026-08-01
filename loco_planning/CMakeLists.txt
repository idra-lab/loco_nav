cmake_minimum_required(VERSION 3.0.2)
project(loco_planning)

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
    scripts/params.py  
    # Controllers  
    scripts/controller.py
    scripts/spawn_controllers.py
    scripts/controllers/lyapunov.py
    # Planners - Base
    scripts/planner_base.py
    # Planners - Voronoi
    scripts/planner_voronoi.py
    scripts/unicycle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  


install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY scripts/utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/geometry
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/controllers
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/planners
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)

 install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
      

