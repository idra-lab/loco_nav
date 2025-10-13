#!/usr/bin/env python
import matplotlib
matplotlib.use('TkAgg')
import rospy
from planners.rrt import RRT
from planner_base import PlannerParamsBase, PlannerBase


class RRTPlannerParams(PlannerParamsBase):
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=1.):
        super().__init__(robot_radius, v_max, curvature_max)

class RRTPlanner(PlannerBase):
    def __init__(self, robot_radius=0.2, v_max=0.1, curvature_max=0.1, robot_name="limo0", debug = False):
        super().__init__(robot_radius, v_max, curvature_max, robot_name="limo0", debug = False)
        self.params = RRTPlannerParams(robot_radius=0.2, v_max=0.1, curvature_max=0.1)
        self.REFERENCE_TYPE = 'PIECE_WISE' #'PIECE_WISE', 'DUBINS' , 'DUBINS_MULTIPOINT'

    def plan_path(self):
        rospy.loginfo("Running RRT...")
        xs = [ox for (ox, _, _) in self.obstacle_list]
        ys = [oy for (_, oy, _) in self.obstacle_list]
        self.rand_area = [min(xs), max(xs)]

        rrt = RRT(
            start=self.start,
            goal=self.goal,
            obstacle_list=self.obstacle_list,
            expand_dis=3. / self.params.Kmax,  # it should be 3 times turning radius to avoid strange loops
            path_resolution=0.25,
            max_iter=1000,
            rand_area=self.rand_area,
            robot_radius=self.params.robot_radius,
            seed=0  # to have always the same solution
        )
        path = rrt.planning(animation=True)
        # Reverse path to start->goal order
        path = path[::-1]
        return path

# ---------- Main ----------
if __name__ == "__main__":
    rospy.init_node("planner_node", anonymous=False) #with anonymous=False ROS will handle killing any old instance automatically.
    planner = PlannerBase(robot_radius=0.2, v_max=0.3, curvature_max=3., robot_name="limo0", debug=False)

    while not rospy.is_shutdown():
        # be sure you have received all messages
        if not planner.computed_path and planner.goal_ready and planner.map_ready and planner.map_ready:
            planner.path = planner.plan_path()
            planning_done = planner.send_path(planner.path)
            rospy.spin()  # keeps the process alive if you call it not in interactive mode
            if planning_done:
                break