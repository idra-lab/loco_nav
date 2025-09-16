#!/usr/bin/env python
import matplotlib
matplotlib.use('TkAgg')
import rospy
from planner_base import PlannerParamsBase, PlannerBase
from planners.voronoi import VoronoiBasePlanner
import math
import numpy as np

class VoronoiPlannerParams(PlannerParamsBase):
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=5.5):
        super().__init__(robot_radius, v_max, curvature_max)

class VoronoiPlanner(PlannerBase):
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=0.1, robot_name="limo0", debug = False):
        super().__init__(robot_radius, v_max, curvature_max, robot_name="limo0", debug = False)
        self.params = VoronoiPlannerParams(robot_radius=0.2, v_max=0.1, curvature_max=0.1)
        self.REFERENCE_TYPE = 'PIECE_WISE' #'PIECE_WISE', 'DUBINS' , 'DUBINS_MULTIPOINT'

    # ovverride callback
    def obstacles_cb(self, msg):
        if self.obstacles_ready:
            return
        for obstacle in msg.obstacles:
            for p in obstacle.polygon.points:
                radius = obstacle.radius
                self.obstacle_list.append((p.x, p.y, radius))
        self.obstacles_ready = True
        rospy.loginfo("Added %d obstacle samples", len(self.obstacle_list))



    def plan_path(self):
        rospy.loginfo("Running Voronoi...")

        self.voronoi_base_planner = VoronoiBasePlanner(start=self.start, goal=self.goal, obstacle_list=self.obstacle_list, robot_radius=self.robot_radius)
        path = self.voronoi_base_planner.planning(show_animation=True)

        return path

# ---------- Main ----------
if __name__ == "__main__":
    rospy.init_node("planner_node", anonymous=False) #with anonymous=False ROS will handle killing any old instance automatically.
    planner = VoronoiPlanner(robot_radius=0.2, v_max=0.1, curvature_max=4.0, robot_name="limo0", debug=False)

    while not rospy.is_shutdown():
        # be sure you have received all messages
        if not planner.computed_path and   planner.obstacles_ready:
            #set custom goal
            planner.goal = np.array([5,0,0.5])
            planner.path = planner.plan_path()
            planning_done = planner.send_path(planner.path)
            if planning_done:
                break