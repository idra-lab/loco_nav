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
    scripts/controllers/lyapunov.py
    scripts/utils/math_tools.py
    scripts/utils/utils.py
    scripts/params.py
    scripts/utils/trajectory_generator.py
    scripts/unicycle.py
    scripts/utils/velocity_generator.py
    scripts/controller.py
    scripts/spawn_controllers.py
    scripts/utils/communication_utils.py
    scripts/planner_base.py
    scripts/planner_rrt.py    
    scripts/planner_voronoi.py
    scripts/planners/rrt.py
    scripts/planners/dubins.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)




install(DIRECTORY scripts/utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

  
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



