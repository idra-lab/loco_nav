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
    scripts/unicycle.py
    # Controllers  
    scripts/controller.py
    scripts/spawn_controllers.py
    scripts/controllers/lyapunov.py
    # Utils
    scripts/utils/velocity_generator.py
    scripts/utils/communication_utils.py
    scripts/utils/trajectory_generator.py
    scripts/utils/math_tools.py
    scripts/utils/utils.py
    # Dubins
    scripts/planners/dubins.py
    # Planners - Base
    scripts/planner_base.py
    # Planners - RRT
    scripts/planner_rrt.py
    scripts/planners/rrt.py
    # Planners - Voronoi
    scripts/planner_voronoi.py
    # Planners - Dubins Multipoint
    scripts/planners/dp.py
    scripts/planners/dp_utils/_viz_mixin.py
    scripts/planners/dp_utils/cell.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  


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


install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)



