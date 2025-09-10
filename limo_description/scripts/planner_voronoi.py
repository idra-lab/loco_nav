#!/usr/bin/env python
import matplotlib
matplotlib.use('TkAgg')
import rospy
from planner_base import PlannerParamsBase, PlannerBase
from planners.voronoi import VoronoiRoadMapPlanner
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

    #Helpers
    def discretize_segment(self, p0, p1, spacing):
        """Return points along segment from p0->p1 inclusive of p0, exclusive of p1."""
        x0, y0 = p0
        x1, y1 = p1
        L = math.hypot(x1 - x0, y1 - y0)
        if L < 1e-9:
            return []
        n = max(1, int(L // spacing))  # number of intervals
        ts = np.linspace(0.0, 1.0, n + 1)[:-1]  # exclude 1.0 to avoid duplicating next edge's start
        xs = x0 + (x1 - x0) * ts
        ys = y0 + (y1 - y0) * ts
        return list(zip(xs, ys))

    def discretize_polygon(self,vertices, spacing):
        """
        vertices: list of (x,y) in order (closed or open).
        Returns a list of sampled (x,y) along the edges.
        """
        pts = []
        m = len(vertices)
        for i in range(m):
            p0 = vertices[i]
            p1 = vertices[(i + 1) % m]
            pts.extend(self.discretize_segment(p0, p1, spacing))
        return pts

    def discretize_circle(self,cx, cy, r, spacing, n_min=12):
        circumference = 2 * math.pi * r
        n = max(n_min, int(circumference // spacing))
        angles = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in angles]

    # ovverride callback
    def obstacles_cb(self, msg):
        if self.obstacles_ready:
            return

        edge_spacing = 0.5 * self.robot_radius
        all_points = []

        for obstacle in msg.obstacles:
            # Cylinder (1 point + radius)
            if len(obstacle.polygon.points) == 1:
                cx = obstacle.polygon.points[0].x
                cy = obstacle.polygon.points[0].y
                radius = float(obstacle.polygon.radius)
                samples = self.discretize_circle(cx, cy, radius, edge_spacing)

            # Polygon
            else:
                verts = [(p.x, p.y) for p in obstacle.polygon.points]
                samples = self.discretize_polygon(verts, edge_spacing)

            all_points.extend(samples)

        self.obstacle_list.extend(all_points)
        self.obstacles_ready = True
        rospy.loginfo("Added %d obstacle samples", len(all_points))

    #override callback
    def map_borders_cb(self, msg):
        if self.map_ready:
            return
        border_points = self.discretize_border(msg, discretization_point=20, radius=0.25)
        self.obstacle_list.extend(border_points)
        self.map_ready = True
        rospy.loginfo("Discretized border added with %d points", len(border_points))

    def plan_path(self):
        rospy.loginfo("Running Voronoi...")

        voronoi_planner = VoronoiRoadMapPlanner(start=self.start, goal=self.goal, obstacle_list=self.obstacle_list, robot_radius=self.robot_radius)
        path = voronoi_planner.planning(show_animation=True)

        return path

# ---------- Main ----------
if __name__ == "__main__":
    rospy.init_node("planner_node", anonymous=False) #with anonymous=False ROS will handle killing any old instance automatically.
    planner = VoronoiPlanner(robot_radius=0.2, v_max=0.1, curvature_max=4.0, robot_name="limo0", debug=False)

    while not rospy.is_shutdown():
        # be sure you have received all messages
        if not planner.computed_path and planner.goal_ready and planner.obstacles_ready and planner.map_ready:
            planner.path = planner.plan_path()
            planning_done = planner.send_path(planner.path)
            if planning_done:
                break