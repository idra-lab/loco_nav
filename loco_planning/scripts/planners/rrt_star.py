"""

Path planning Sample Code with RRT*

author: Atsushi Sakai(@Atsushi_twi)

"""

import math
import sys
import matplotlib
matplotlib.use('TkAgg')        # select interactive backend (TkAgg) for animations
import matplotlib.pyplot as plt # plotting
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from planners.rrt import RRT
import random
show_animation = True
import numpy as np
import time
from math import sqrt, atan2, sin, cos

class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0 # added cost

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0,
                 search_until_max_iter=False,
                 robot_radius=0.0,
                 seed = None):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter,
                         robot_radius=robot_radius)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = []
        if seed is not None:
            random.seed(seed)

    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + math.hypot(new_node.x-near_node.x,              new_node.y-near_node.y)

            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation:
                self.draw_graph(rnd)

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        safe_goal_costs = [self.node_list[i].cost +
                           self.calc_dist_to_goal(self.node_list[i].x, self.node_list[i].y)
                           for i in safe_goal_inds]

        min_cost = min(safe_goal_costs)
        for i, cost in zip(safe_goal_inds, safe_goal_costs):
            if cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(
                edge_node, self.obstacle_list, self.robot_radius)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                for node in self.node_list:
                    if node.parent == self.node_list[i]:
                        node.parent = edge_node
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(self.node_list[i])

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    @staticmethod
    def planning_map(start, goal, map, resolution=0.5, subcell_sampling_factor=0.25,
                     step_size=0.4, max_iter=5000, goal_sample_rate=0.05, mu=1.0):
        """
        RRT* in continuous space using a discrete occupancy grid. Returns path + path length.
        Visualization similar to RRT for teaching purposes.

        Parameters:
            start, goal: [y, x] in meters
            map: boolean 2D array (True = obstacle)
            resolution: meters per grid cell
            step_size: expansion step (meters)
            max_iter: maximum number of iterations
            goal_sample_rate: probability of sampling the goal
            radius: neighborhood radius for rewiring (meters)

        Returns:
            path: list of (y, x) nodes in meters
            path_length: total length of the path (meters)
        """
        rows, cols = map.shape
        width = cols * resolution
        height = rows * resolution

        plt.imshow(map, cmap='gray_r', origin='lower', extent=[0, width, 0, height])
        plt.plot(start[0],start[1],  'or', markersize=10)
        plt.plot(goal[0], goal[1],  'oy', markersize=10)
        plt.ion()

        nodes = [tuple(start)]
        parent = {tuple(start): None}
        cost = {tuple(start): 0.0}

        def distance(p1, p2):
            return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

        def sample():
            if random.random() < goal_sample_rate:
                return tuple(goal)
            return (random.uniform(0, height), random.uniform(0, width))

        # get nearest point in the graph to q_rand
        def nearest(q_rand):
            return min(nodes, key=lambda n: distance(n, q_rand))

        # expand from q_nearest to q_rand
        def extend(q_nearest, q_rand):
            theta = atan2(q_rand[0] - q_nearest[0], q_rand[1] - q_nearest[1])
            q_new = (q_nearest[0] + step_size * sin(theta),
                     q_nearest[1] + step_size * cos(theta))
            q_new = (max(0, min(q_new[0], height)), max(0, min(q_new[1], width)))
            return q_new

        def is_collision_free(q_nearest, q_new):
            """Check if the line between q_nearest and q_new crosses an obstacle."""
            # get the number of subsample for collision checking on the edge between q_nearest and q_rand
            n = int(distance(q_nearest, q_new) / (resolution * subcell_sampling_factor))
            if n == 0:  # the point is inside the ball of radius of the resolution so check the point right away
                ry, cx = int(q_new[1] / resolution), int(q_new[0] / resolution)
                # discard points outside the map
                if ry < 0 or cx < 0 or ry >= rows or cx >= cols:
                    return False
                # discard points on obstacles (positive map)
                if map[ry, cx]:
                    return False
            else:
                for i in range(n + 1):
                    y = q_nearest[1] + (q_new[1] - q_nearest[1]) * i / n
                    x = q_nearest[0] + (q_new[0] - q_nearest[0]) * i / n
                    ry, cx = int(y / resolution), int(x / resolution)
                    # discard points outside the map
                    if ry < 0 or cx < 0 or ry >= rows or cx >= cols:
                        return False
                    # discard points on obstacles (positive map)
                    if map[ry, cx]:
                        return False
            return True

        def near(q_new):
            return [n for n in nodes if distance(n, q_new) < mu]

        # --- Main RRT* Loop ---
        for i in range(max_iter):
            q_rand = sample()
            q_nearest = nearest(q_rand)
            q_new = extend(q_nearest, q_rand)
            # if there is a collision continue sampling a new node
            if not is_collision_free(q_nearest, q_new):
                continue

            # --- Choose best parent to q_new---
            near_nodes = near(q_new)
            #compute cost for connecting q_new from q_nearest
            q_min = q_nearest
            c_min = cost[q_nearest] + distance(q_nearest, q_new)
            #Parent selection: loop through  near nodes to see if there is a lower cost parent
            for q_near in near_nodes:
                if is_collision_free(q_near, q_new):
                    c_new = cost[q_near] + distance(q_near, q_new)
                    if c_new < c_min:
                        q_min = q_near
                        c_min = c_new
            #add the node q_new to the graph setting its parent q_min
            nodes.append(q_new)
            parent[q_new] = q_min
            cost[q_new] = c_min

            # Rewiring: find if among q_near nodes there is a lower cost connection through q_new which is collision free
            for q_near in near_nodes:
                if is_collision_free(q_new, q_near):
                    c_through_new = cost[q_new] + distance(q_new, q_near)
                    if c_through_new < cost[q_near]:
                        # rewire setting as parent node of q_near the q_new
                        parent[q_near] = q_new
                        cost[q_near] = c_through_new
                        #plot the rewired edge
                        plt.plot( [q_near[0], q_new[0]], [q_near[1], q_new[1]],'b--', alpha=1.)

            # Visualization of added node q_new with the added edge
            plt.plot([parent[q_new][0], q_new[0]], [parent[q_new][1], q_new[1]],color='g', linewidth=1.3, alpha=0.7)
            plt.plot( q_new[0], q_new[1],'g*', markersize=5)
            plt.pause(0.0001)

            # Check for goal
            if distance(q_new, goal) < step_size * 2:
                parent[tuple(goal)] = q_new
                cost[tuple(goal)] = cost[q_new] + distance(q_new, goal)
                print(f"Goal reached at iteration {i}")
                break

        # --- Path reconstruction ---
        if tuple(goal) not in parent:
            print("Cannot find path, number of iterations exceeded")
            plt.ioff()
            plt.show()
            return None, None

        path = []
        node = tuple(goal)
        while node is not None:
            path.append(node)
            node = parent[node]
        path.reverse()

        # --- Compute total path length ---
        path_length = sum(distance(p1, p2) for p1, p2 in zip(path[:-1], path[1:]))

        # --- Final path visualization ---
        for p1, p2 in zip(path[:-1], path[1:]):
            plt.plot([p1[0], p2[0]],[p1[1], p2[1]],  'r-', linewidth=3.0)
        for p in path:
            plt.plot(p[0], p[1],  'r.', markersize=4)

        plt.ioff()
        plt.show()

        return path, path_length


if __name__ == '__main__':
    start = [1., 1.]
    goal = [9., 9.]
    np.random.seed(0)
    map_width = 10.
    map_height = 10.
    resolution = 0.5

    map = np.random.rand(int(map_height / resolution), int(map_width / resolution)) < 0.1

    t0 = time.time()
    path, path_length = RRTStar.planning_map(start=start, goal=goal, map=map, resolution=resolution)
    t1 = time.time()

    if path is not None:
        print(f"Path found! length={path_length:.3f} m, runtime={t1 - t0:.3f} s")
    else:
        print("No path found.")