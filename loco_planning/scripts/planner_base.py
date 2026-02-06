#!/usr/bin/env python
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import rospy
import math
from geometry_msgs.msg import Polygon, PoseArray
from obstacles_msgs.msg import ObstacleArrayMsg
from loco_planning.msg import Reference   # <-- your custom message
from planners.rrt import RRT
from planners.rrt_star import RRTStar
import params as conf
from termcolor import  colored
import sys
import rosnode
import threading
from  utils.communication_utils import getInitialStateFromOdom, checkRosMaster, launchFileNode
import numpy as np
from planners.dubins import dubins_shortest_path, get_discretized_path_from_dubins, plotdubins
from planners.dp import DP
import os

class PlannerParamsBase:
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=1.):
        self.robot_radius = robot_radius
        self.v_max = v_max  # desired linear velocity
        self.Kmax = curvature_max

class PlannerBase:
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=3., robot_name="limo0", debug = False):
        self.robot_radius = robot_radius
        self.robot_name = robot_name

        self.params = PlannerParamsBase(robot_radius, v_max, curvature_max)
        self.dt = conf.robot_params[self.robot_name]['dt']     # time step for discretization
        self.REFERENCE_TYPE = 'DUBINS_MULTIPOINT' #'PIECE_WISE', 'DUBINS' , 'DUBINS_MULTIPOINT'
        self.DEBUG = debug

        self.obstacle_list = []
        self.obstacles_ready = False
        self.map_ready = False
        self.goal_ready = False
        self.computed_path = False

    def ros_init(self, start_simulation=False, regenerate_map=True, launch_file=None):
        if start_simulation:
            self.startSimulation(regenerate_map=regenerate_map, launch_file=launch_file)
        rospy.init_node("planner_node", anonymous=False)  # with anonymous=False ROS will handle killing any old instance automatically.

        # ROS interfaces
        # check if controller node is running
        nodes = rosnode.get_node_names()
        node_name = f"/{self.robot_name}/controller"
        if node_name in nodes:
            print(f"Node {node_name} exists!")
            p0_x, p0_y, yaw0 = getInitialStateFromOdom(self.robot_name)
            self.start = [p0_x, p0_y, yaw0]
        else:
            print(colored(f"Node {node_name} not found. Please make sure you started the controller", "red"))
            sys.exit()

        rospy.Subscriber("/map_borders", Polygon, self.map_borders_cb)
        rospy.Subscriber("/obstacles", ObstacleArrayMsg, self.obstacles_cb)
        rospy.Subscriber("/gates", PoseArray, self.goal_cb)
        self.ref_pub = rospy.Publisher("/" + self.robot_name + "/ref", Reference, queue_size=10)
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
            #already cylinder
            if len(obstacle.polygon.points) == 1:
                xs = obstacle.polygon.points.x
                ys = obstacle.polygon.points.y
                radius = obstacle.radius
            else:#find outscribed circle to polygon
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
            #extracting yaw orientation from quaternion as arctan2(2(wz+xy),1−2(y2+z2))
            gate_t0 = 2.0 * (goal.orientation.w * goal.orientation.z + goal.orientation.x * goal.orientation.y)
            gate_t1 = 1.0 - 2.0 * (goal.orientation.y * goal.orientation.y + goal.orientation.z * goal.orientation.z)
            gate_yaw = math.atan2(gate_t0, gate_t1)
            self.goal = (goal.position.x, goal.position.y, gate_yaw)
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

    def plan_path(self):
        rospy.loginfo("Running RRT...")
        xs = [ox for (ox, _, _) in self.obstacle_list]
        ys = [oy for (_, oy, _) in self.obstacle_list]
        self.rand_area = [min(xs), max(xs)]

        rrt = RRT(
            start=self.start,
            goal=self.goal,
            obstacle_list=self.obstacle_list,
            expand_dis=3./self.params.Kmax, # it should be 3 times turning radius to avoid strange loops
            path_resolution=0.25,
            max_iter= 1000,
            rand_area=self.rand_area,
            robot_radius=self.params.robot_radius,
            seed=0   # to have always the same solution
        )
        path = rrt.planning(animation=True)
        # Reverse path to start->goal order
        path = path[::-1]
        return path

    def send_path(self, path):
        if path is None:
            rospy.logwarn("No path found.")
        else:

            rospy.loginfo("Path found with %d waypoints.", len(path))

            # ---- Plot first (non-blocking) ----
            plt.plot([x for (x, y) in path],
                     [y for (x, y) in path], '-r')
            plt.grid(True)
            # draw immediately
            plt.draw()
            plt.pause(0.01)
            plt.show(block=False)


            # compute reference from path
            self.reference = self.computeReferenceFromPath(path)
            if self.DEBUG:
                plt.plot(self.reference[:, 0], self.reference[:, 1], "+k")
            threading.Thread(target=self.publish_reference, args=(self.reference,), daemon=True).start()
        return True

    def computeReferenceFromPath(self, path):
        """
        Discretize path with velocity self.params.v_max and dt
        """
        if self.REFERENCE_TYPE == 'PIECE_WISE':
            theta0 = self.start[2]
            reference = []
            ds = self.params.v_max * self.dt
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
                    theta_d = theta0 + t * (theta1 - theta0)
                    reference.append(np.array([x_d, y_d, theta_d, self.params.v_max, 0]))
                theta0 = theta1

            if len(reference) > 1:
                reference = np.vstack(reference)
            else:
                print(colored("you are already in target","red"))
                sys.exit()

        elif self.REFERENCE_TYPE == 'DUBINS':
            curve,curvatures, lengths  = dubins_shortest_path(self.start[0], self.start[1], self.start[2], self.goal[0], self.goal[1], self.goal[2], self.params.Kmax)
            plotdubins(curve, color1='r', color2='g', color3='b')
            x_ref, y_ref, theta_ref, v_ref, omega_ref, time = get_discretized_path_from_dubins(self.start, self.params.v_max, curve,lengths, self.dt)
            reference = np.vstack([x_ref, y_ref, theta_ref, v_ref, omega_ref, time]).T

        elif self.REFERENCE_TYPE == 'DUBINS_MULTIPOINT':
            rospy.logdebug("Computing Dubins multipoint reference...")
            points = path
            def_thetas = [self.start[2]] + [0.0]*(len(points)-2) + [self.goal[2]]
            fixed_angles = [True] + [False]*(len(points)-2) + [True]
            k_max = self.params.Kmax
            
            dp_instance = DP(points, fixed_angles, k_max, discretizations=90, refinements=0, def_thetas=def_thetas)
            opt_angles = dp_instance.solve_dp()

            for i in range(len(points)-1):
                p_start = points[i]
                p_end = points[i+1]
                start_theta = opt_angles[i]
                end_theta = opt_angles[i+1]
                curve, curvatures, lengths  = dubins_shortest_path(p_start[0], p_start[1], start_theta, p_end[0], p_end[1], end_theta, 3)
                plotdubins(curve, color1='r', color2='g', color3='b', show=False)
                x_ref, y_ref, theta_ref, v_ref, omega_ref, time = get_discretized_path_from_dubins((p_start[0], p_start[1], start_theta), self.params.v_max, curve,lengths, self.dt)
                if i==0:
                    reference = np.vstack([x_ref, y_ref, theta_ref, v_ref, omega_ref, time]).T
                else:
                    reference = np.vstack([reference, np.vstack([x_ref, y_ref, theta_ref, v_ref, omega_ref, time]).T])
            plt.show(block=False)
           
            rospy.logdebug("Dubins multipoint reference computed with %d points.", reference.shape[0])


        else:
            print(colored("Wrong ref. type","red"))
        return reference

    def publish_reference(self, reference):
        """
         publish Reference messages
        """
        rospy.loginfo("Publishing reference trajectory...")
        try:
            for sample in range(len(reference)):
                ref = Reference()
                ref.x_d = reference[sample,0]
                ref.y_d = reference[sample,1]
                ref.theta_d = reference[sample,2]
                ref.v_d = reference[sample,3]
                ref.omega_d =reference[sample,4]

                self.ref_pub.publish(ref)
                if self.DEBUG:
                    print(f"Reference: {ref.x_d}, {ref.y_d}, {ref.theta_d}")

                #TODO do it with timer rather than with threads https://chatgpt.com/c/68b5915d-3b40-832b-a7b3-ad92f88926b7
                try:
                    self.rate.sleep()  # sleep per point
                except rospy.ROSInterruptException:
                    return
        finally:
            rospy.loginfo("Reference trajectory published.")
            ref.plan_finished = True
            self.ref_pub.publish(ref)
            rospy.signal_shutdown("Finished RRT planning")
        self.computed_path = True

    def startSimulation(self, regenerate_map=True, launch_file=None):
        os.system("pkill rosmaster")
        os.system("pkill gzserver")
        os.system("pkill gzclient")
        os.system("pkill rviz")
        checkRosMaster()
        additional_args=["start_controller:=true"]
        if not regenerate_map:
            additional_args.append("generate_new_config:=false")
        if launch_file is None:
            launchFileNode(package="loco_planning", launch_file="multiple_robots.launch", additional_args=additional_args)
        else:
            launchFileNode(package="loco_planning", launch_file=launch_file, additional_args=additional_args)
        rospy.sleep(6.)
# ---------- Main ----------
if __name__ == "__main__":
    planner = PlannerBase(robot_radius=0.2, v_max=0.3, curvature_max=1/0.35, robot_name="limo0", debug=False)
    # to test RRT /RRTStar planner.ros_init(start_simulation=True, regenerate_map=False)
    planner.ros_init(start_simulation=True, regenerate_map=False)
    while not rospy.is_shutdown():
        # be sure you have received all messages
        if not planner.computed_path and planner.goal_ready and planner.map_ready and planner.map_ready:
            planner.path = planner.plan_path()
            planning_done = planner.send_path(planner.path)
            rospy.spin()  # keeps the process alive if you call it not in interactive mode
            if planning_done:
                break