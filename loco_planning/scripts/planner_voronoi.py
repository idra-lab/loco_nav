#!/usr/bin/env python
import matplotlib
matplotlib.use('TkAgg')
import rospy
from planner_base import PlannerParamsBase, PlannerBase
from planners.voronoi import VoronoiBasePlanner
import numpy as np

class VoronoiPlannerParams(PlannerParamsBase):
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=5.5):
        super().__init__(robot_radius, v_max, curvature_max)

class VoronoiPlanner(PlannerBase):
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=0.1, robot_name="limo0", debug = False):
        super().__init__(robot_radius, v_max, curvature_max, robot_name="limo0", debug = False)
        self.params = VoronoiPlannerParams(robot_radius=0.2, v_max=0.1, curvature_max=0.1)
        self.REFERENCE_TYPE = 'DUBINS_MULTIPOINT' #'PIECE_WISE', 'DUBINS' , 'DUBINS_MULTIPOINT'

    #override unused callbacks
    def map_borders_cb(self, msg):
        pass

    # override unused callbacks
    def goal_cb(self, msg):
        pass

    #override obstacles_cb  (the only published by roslaunch loco_planning labyrinth.launch start_controller:=true)
    def obstacles_cb(self, msg):
        if self.obstacles_ready:
            return
        for obstacle in msg.obstacles:
            for p in obstacle.polygon.points:
                self.obstacle_list.append([p.x, p.y])
        self.obstacles_ready = True
        rospy.loginfo(f"Added {len(self.obstacle_list)} point obstacles")


    # override callback
    def plan_path(self):

        self.voronoi_base_planner = VoronoiBasePlanner(start=self.start, goal=self.goal, obstacle_list=self.obstacle_list, robot_radius=self.robot_radius)
        path = self.voronoi_base_planner.planning(show_animation=True)

        return path

# ---------- Main ----------
if __name__ == "__main__":
    planner = VoronoiPlanner(robot_radius=0.2, v_max=0.1, curvature_max=4.0, robot_name="limo0", debug=False)
    planner.ros_init()

    #to test launch: roslaunch loco_planning labyrinth.launch start_controller:=true
    # this will publish everything as obstacles discretized (i.e. no map_borders)
    planner.goal = np.array([5., 0.])

    while not rospy.is_shutdown():
        # be sure you have received all messages
        if not planner.computed_path and planner.obstacles_ready:
            planner.path = planner.plan_path()
            planning_done = planner.send_path(planner.path)
            rospy.spin()  # keeps the process alive if you call it not in interactive mode
            if planning_done:
                break