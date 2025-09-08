"""
Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

"""

import math                    # math functions (sqrt, hypot, cos, sin, atan2, etc.)
import random                  # random sampling utilities
import matplotlib
matplotlib.use('TkAgg')        # select interactive backend (TkAgg) for animations
import matplotlib.pyplot as plt # plotting
import numpy as np             # numerical helpers (used here for deg->rad conversion)
import sys
show_animation = True          # toggle for live visualization


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """
        def __init__(self, x, y):
            self.x = x              # node position x
            self.y = y              # node position y
            self.path_x = []        # discretized x coordinates from parent to this node
            self.path_y = []        # discretized y coordinates from parent to this node
            self.parent = None      # pointer to parent Node in the tree

    class AreaBounds:
        # Helper to store rectangular play area bounds
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius
        """
        self.start = self.Node(start[0], start[1])     # start node
        self.end = self.Node(goal[0], goal[1])         # goal node (used for sampling/termination)
        self.min_rand = rand_area[0]                   # sampling min bound (both axes)
        self.max_rand = rand_area[1]                   # sampling max bound (both axes)
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area) # rectangular constraint region
        else:
            self.play_area = None
        self.expand_dis = expand_dis                   # max extension per step towards a sample
        self.path_resolution = path_resolution         # discretization step along an edge
        self.goal_sample_rate = goal_sample_rate       # % chance (0–100) to sample the goal directly
        self.max_iter = max_iter                       # maximum iterations to attempt
        self.obstacle_list = obstacle_list             # list of circular obstacles (x, y, radius)
        self.node_list = []                            # storage for all nodes in the tree
        self.robot_radius = robot_radius               # inflate obstacles by robot radius

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """
        self.node_list = [self.start]                  # initialize tree with the start node
        for i in range(self.max_iter):                 # main RRT growth loop
            print(f"Iter: {i}")                       # simple progress print
            rnd_node = self.get_random_node()          # sample a random config (goal with some probability)
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)  # index of nearest tree node
            nearest_node = self.node_list[nearest_ind] # that nearest node

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)       # extend towards the sample

            # add the node only if it's inside play area (if defined) and collision-free
            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            # optionally draw the intermediate tree every 5 iters (including i == 0)
            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            # check if we got close enough to the goal to attempt a final connection
            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)  # try to connect
                if self.check_collision(final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)  # reconstruct path

            # draw on the other iterations (i % 5 != 0) as well
            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path within max_iter

    def steer(self, from_node, to_node, extend_length=float("inf")):
        """
        Create a new node by moving from 'from_node' towards 'to_node'
        up to 'extend_length', discretized by path_resolution.
        """
        new_node = self.Node(from_node.x, from_node.y)     # start at parent position
        d, theta = self.calc_distance_and_angle(new_node, to_node)  # distance & direction to target

        new_node.path_x = [new_node.x]                     # seed the path samples with start point
        new_node.path_y = [new_node.y]

        if d < extend_length:                              # don't overshoot the target
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)  # number of discretization steps

        for _ in range(n_expand):                          # march along the ray towards to_node of path_resolution
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)       # remaining distance after stepping
        if d <= self.path_resolution:                      # snap to exact target if within one step
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node                        # set tree linkage

        return new_node

    def generate_final_course(self, goal_ind):
        """
        Backtrack from node_list[goal_ind] to start via parent links to produce a path.
        """
        path = [[self.end.x, self.end.y]]                  # start with the goal position
        node = self.node_list[goal_ind]                    # begin backtracking at the last node
        while node.parent is not None:                     # follow parents to the root
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])                      # include the start node

        return path                                        # path is from goal to start (caller may plot directly)

    def calc_dist_to_goal(self, x, y):
        """
        Euclidean distance from (x, y) to the stored goal.
        """
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        """
        With probability (100 - goal_sample_rate)%, sample uniformly in the space.
        Otherwise, return the goal node (goal bias).
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        """
        Clear and redraw the current tree, obstacles, bounds, and start/goal.
        Optionally show the latest random sample.
        """
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [sys.exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")                  # plot sample as black triangle
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')  # visualize robot footprint
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")  # each edge as a green polyline

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)                # draw circular obstacles in blue

        if self.play_area is not None:
            # draw rectangle of allowed area
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")        # start as red x
        plt.plot(self.end.x, self.end.y, "xr")            # goal as red x
        plt.axis("equal")                                 # equal scale x/y
        plt.axis([self.min_rand, self.max_rand,           # set visible window to sampling area
                  self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)                                   # brief pause to render

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        """
        Draw a circle at (x, y) with radius 'size' using a polyline.
        """
        deg = list(range(0, 360, 5))       # sample every 5 degrees
        deg.append(0)                      # close the loop
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        """
        Return index of node in 'node_list' that is closest (squared distance) to 'rnd_node'.
        """
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))   # argmin
        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):
        """
        Return True if either no play_area is set, or node lies inside play_area.
        """
        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True   # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):
        """
        Discrete collision check: ensure all samples along node.path are outside
        each obstacle inflated by robot_radius.
        """
        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]                 # vector x from path points to obstacle center
            dy_list = [oy - y for y in node.path_y]                 # vector y from path points to obstacle center
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]  # squared distances

            if min(d_list) <= (size + robot_radius) ** 2:           # any sample inside inflated obstacle?
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        """
        Compute Euclidean distance and heading angle from 'from_node' to 'to_node'.
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

if __name__ == '__main__':

    # Goal
    gx=6.0
    gy=10.0
    print("start " + __file__)  # show which file is running (useful when imported)

    # ====Search Path with RRT====
    # obstacles are all cylindric with centerx, centery, radius tuples
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius] circular obstacles
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],                 # starting coordinate
        goal=[gx, gy],                # goal coordinate (passed as args)
        rand_area=[-2, 15],           # uniform sampling square in both axes
        obstacle_list=obstacleList,   # obstacles
        # play_area=[0, 10, 0, 14]    # uncomment to constrain to a rectangle
        robot_radius=0.8              # robot radius for obstacle inflation
        )
    path = rrt.planning(animation=show_animation)  # run planner

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()                                  # draw last tree state
            plt.plot([x for (x, y) in path],                  # plot path x's
                     [y for (x, y) in path], '-r')            # plot path y's (red polyline)
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac (ensures render)
            plt.show()       # keep the plot window open

