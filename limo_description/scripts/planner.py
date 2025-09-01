#!/usr/bin/env python
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import rospy
import math
from geometry_msgs.msg import Polygon, PoseArray
from obstacles_msgs.msg import ObstacleArrayMsg
from limo_description.msg import Reference   # <-- your custom message
from rrt import RRT
import params as conf
from termcolor import  colored
import sys
import rosnode
import threading
from communication_utils import getInitialStateFromOdom

class Planner:
    def __init__(self, robot_radius=0.2, v_d=0.1, robot_name="limo0", debug = False):
        self.robot_name = robot_name
        self.robot_radius = robot_radius
        self.v_d = v_d   # desired linear velocity
        self.dt = conf.robot_params[self.robot_name]['dt']     # time step for discretization
        self.DEBUG = debug
        # chec if controller is running
        nodes = rosnode.get_node_names()

        node_name = f"/{self.robot_name}/controller"
        if node_name in nodes:
            print(f"Node {node_name} exists!")
            p0_x, p0_y, yaw0 = getInitialStateFromOdom(self.robot_name)
            self.start = [p0_x, p0_y, yaw0]
        else:
            print(colored(f"Node {node_name} not found. Please make sure you started the controller", "red"))
            sys.exit()


        self.obstacle_list = []
        self.obstacles_ready = None
        self.map_ready = False
        self.goal_ready = False
        self.computed_path = False

        # ROS interfaces
        rospy.Subscriber("/map_borders", Polygon, self.map_borders_cb)
        rospy.Subscriber("/obstacles", ObstacleArrayMsg, self.obstacles_cb)
        rospy.Subscriber("/gates", PoseArray, self.goal_cb)
        self.ref_pub = rospy.Publisher("/"+self.robot_name+"/ref", Reference, queue_size=10)
        self.rate = rospy.Rate(1 / self.dt)  # 100Hz loop, adjust as needed
        rospy.loginfo("Planner initialized. Waiting for map, obstacles, and goal...")



    # ---------- Callbacks ----------
    def map_borders_cb(self, msg):
        if self.map_ready:
            return
        border_points = self.discretize_border(msg, discretization_point=20, radius=0.25)
        self.obstacle_list.extend(border_points)
        self.map_ready = True
        rospy.loginfo("Discretized border added with %d points", len(border_points))

    def obstacles_cb(self, msg):
        if self.obstacles_ready:
            return
        for obstacle in msg.obstacles:
            xs = [p.x for p in obstacle.polygon.points]
            ys = [p.y for p in obstacle.polygon.points]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            #find outscribed circle
            radius = max(math.hypot(p.x - cx, p.y - cy) for p in obstacle.polygon.points)
            self.obstacle_list.append((cx, cy, radius))
            rospy.loginfo("Added polygon obstacle at (%.2f, %.2f), r=%.2f", cx, cy, radius)
        self.obstacles_ready = True

    def goal_cb(self, msg):
        if self.goal_ready:
            return
        for goal in msg.poses:
            self.goal = (goal.position.x, goal.position.y)
            self.goal_ready = True
            rospy.loginfo("Goal set: %s", self.goal)

    # ---------- Helpers ----------
    def discretize_border(self, polygon_msg, discretization_point=20, radius=0.01):
        points = []
        pts = polygon_msg.points
        for i in range(len(pts)):
            start = pts[i]
            end = pts[(i + 1) % len(pts)]
            for j in range(discretization_point):
                x = start.x + (end.x - start.x) * float(j) / discretization_point
                y = start.y + (end.y - start.y) * float(j) / discretization_point
                points.append((x, y, radius))
        return points

    def try_run_rrt(self):

        rospy.loginfo("Running RRT...")
        xs = [ox for (ox, _, _) in self.obstacle_list]
        ys = [oy for (_, oy, _) in self.obstacle_list]
        self.rand_area = [min(xs), max(xs)]

        rrt = RRT(
            start=self.start,
            goal=self.goal,
            obstacle_list=self.obstacle_list,
            expand_dis=1.0,
            path_resolution=0.25,
            max_iter= 1000,
            rand_area=self.rand_area,
            robot_radius=self.robot_radius
        )
        self.path = rrt.planning(animation=True)

        if self.path is None:
            rospy.logwarn("No path found.")
        else:
            rospy.loginfo("Path found with %d waypoints.", len(self.path))
            # ---- Plot first (non-blocking) ----
            plt.plot([x for (x, y) in self.path],
                     [y for (x, y) in self.path], '-r')
            plt.grid(True)
            plt.draw()
            plt.show(block=False)
            threading.Thread(target=self.publish_reference, args=(self.path,), daemon=True).start()


        return True

    def publish_reference(self, path):
        """
        Discretize path with velocity v_d and dt, publish Reference messages
        """
        rospy.loginfo("Publishing reference trajectory...")

        # Reverse path to start->goal order
        path = path[::-1]

        theta0 = self.start[2]
        ds = self.v_d * self.dt
        try:
            for i in range(len(path) - 1):
                x0, y0 = path[i]
                x1, y1 = path[i + 1]
                dx = x1 - x0
                dy = y1 - y0
                seg_len = math.hypot(dx, dy)
                theta1 = math.atan2(dy, dx)
                # lnear interpolation
                n_points = max(1, int(seg_len / ds))
                for j in range(n_points):
                    t = float(j) / n_points
                    x_d = x0 + t * dx
                    y_d = y0 + t * dy

                    ref = Reference()
                    ref.x_d = x_d
                    ref.y_d = y_d
                    ref.theta_d = theta0 + t * (theta1 - theta0)
                    ref.v_d = self.v_d
                    ref.omega_d = 0.0  # could be refined with curvature

                    self.ref_pub.publish(ref)
                    if self.DEBUG:
                        print(f"Reference: {ref.x_d}, {ref.y_d}, {ref.theta_d}")
                    try:
                        self.rate.sleep()  # sleep per point
                    except rospy.ROSInterruptException:
                        return
                theta0 = theta1
        finally:
            rospy.loginfo("Reference trajectory published.")
            ref.plan_finished = True
            self.ref_pub.publish(ref)
            rospy.signal_shutdown("Finished RRT planning")
        self.computed_path = True


# ---------- Main ----------
if __name__ == "__main__":
    rospy.init_node("planner_node", anonymous=False) #with anonymous=False ROS will handle killing any old instance automatically.
    planner = Planner(  robot_radius=0.2, v_d=0.1, robot_name="limo0", debug=True)

    while not rospy.is_shutdown():
        # be sure you have received all messages
        if not planner.computed_path and planner.goal_ready and planner.map_ready and planner.map_ready:
            planning_done = planner.try_run_rrt()
            if planning_done:
                break