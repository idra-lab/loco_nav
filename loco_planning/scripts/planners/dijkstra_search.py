"""

Dijkstra Search library

author: Atsushi Sakai (@Atsushi_twi)

"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import random
import time
from heapq import heappush, heappop
from utils.utils import get8NeighborsCost
from collections import defaultdict


class DijkstraSearch:
    class Node:
        """
        Node class for dijkstra search
        """

        def __init__(self, x, y, dist=None, parent=None, edge_ids=None):
            # Node object stores extra info (coordinates, accumulated dist, parent pointer)
            self.x = x
            self.y = y
            self.dist = dist
            self.parent = parent
            self.edge_ids = edge_ids

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.dist) + "," + str(self.parent)

    def __init__(self, show_animation, verbose=False):
        self.show_animation = show_animation
        self.verbose = verbose


    def search_graph(self, start, nodes, edge_ids_list_w, goal=None, use_heap=False):

        """
            Simplified Dijkstra's algorithm. static method (all nodes in open set at ∞) is simpler mathematically, but wastes space if the graph is huge and sparse.
            start node
            nodes: vertices of the graph
            edge_ids_list : adjacency list (list of neighbors for each node index) with weight for each edge
            goal node
        """

        # --- Step 1: initialize distances and parents ---
        dist = {node: math.inf for node in nodes} # set all nodes distances to infinity
        dist[start] = 0.0  # except the start for which is zero
        parent = {node: None for node in nodes}  # data structure to store the id of the predecessor for each node for shortest path

        # --- Step 2: unvisited and visited list
        if use_heap:
            Q = [(0.0, start)]
        else:
            Q = nodes.copy()  # the indexes of all nodes initially unvisited
        visited = []  # list to store visited nodes, empty
        iter = 0
        # --- Step 3: main loop ---
        while Q:  # while queue not empty
            iter+=1
            # select the element of Q with minimum stored distance
            if use_heap:
                _, u = heappop(Q)
            else:
                u = self.mindistance(Q, dist)
                Q.remove(u)

            # Skip if already visited
            if u in visited:
                continue
            visited.append(u)  # mark as visited
            if self.verbose:
                input(f"\niter: {iter}, Press Enter to continue...")
                print("Q:",Q)
                print("dist:", dist)
                print("visited: ", visited)
            # check neighbours
            for v, w in edge_ids_list_w[u]:
                # v = neighbour index
                # w = edge weight
                if (dist[u] + w) < dist[v]:
                    dist[v] =  (dist[u] + w)
                    #add traceback parent
                    parent[v] = u
                    if use_heap:
                        heappush(Q, ((dist[u] + w), v))

        # --- Step 5: reconstruct path ---
        if goal is not None:
            path = self.compute_path(start, goal, parent)
            if self.verbose:
                print(f"shortest path from {start} to {goal} is :  {path}")
        else: # print all the shortest paths to each node
            for node in nodes:
                if node is not start:
                    path = self.compute_path(start, node, parent)
                    if self.verbose:
                        print(f"shortest path from {start} to {node} is :  {path}")

    def compute_path(self, start, goal, parent):
        path = []
        u = goal
        if parent[u] is not None or u == start:  # path exists
            while u is not None:
                path.append(u)
                u = parent[u]
        # invert since we started the list with goal
        path.reverse()
        return path

    def mindistance(self, Q, dist):
        # Start by assuming the first element of Q is the best
        u = Q[0]
        min_dist = dist[u]

        # Loop through all other nodes in Q
        for candidate in Q[1:]:

            if dist[candidate] < min_dist:
                min_dist = dist[candidate]
                u = candidate
        return u

    def search_static(self,  sx, sy, gx, gy, node_x, node_y, edge_ids_list):

        """
            Simplified Dijkstra's algorithm. static method (all nodes in open set at ∞) is simpler mathematically, but wastes space if the graph is huge and sparse.
            sx, sy : start coordinates
            gx, gy : goal coordinates
            node_x, node_y : lists of node coordinates
            edge_ids_list : adjacency list (list of neighbors for each node index)
        """

        plt.plot(gx, gy, 'oy')  # puts a yellow asterisk at the goal
        plt.plot(sx, sy, 'or')  # puts a red asterisk at the start
        plt.ion()  # turns 'interactive mode' on

        # number of nodes in the graph
        N = len(node_x)

        # --- Step 1: find indices of start and goal ---
        start_id = self.find_id(node_x, node_y, self.Node(sx, sy, 0.0, -1))
        goal_id = self.find_id(node_x, node_y, self.Node(gx, gy, 0.0, -1))

        # --- Step 2: initialize distances and parents ---
        dist = {i: math.inf for i in range(N)} # set all nodes distances to infinity
        dist[start_id] = 0.0  # except the start for which is zero
        parent = {i: None for i in range(N)}  # data structure to store the id of the predecessor for each node for shortest path

        # --- Step 3: unvisited and visited list
        Q = list(range(N)) # the indexes of all nodes initially unvisited
        visited = []  # list to store visited nodes, empty

        # --- Step 4: main loop ---
        while Q:  # while queue not empty
            # select element of Q with minimum stored distance
            #u = min(Q, key=lambda x: dist[x])
            u = self.mindistance(Q, dist)
            plt.plot(node_x[u], node_y[u], 'b.')
            # ✅ draw immediately even if non-blocking
            plt.draw()
            plt.pause(0.001)
            plt.show(block=False)

            Q.remove(u)
            visited.append(u)  # mark as visited

            # Stop early if we reached the goal
            if u == goal_id:
                break
            # check neighbours
            for v in edge_ids_list[u]:
                dx = node_x[v] - node_x[u]
                dy = node_y[v] - node_y[u]
                w = math.hypot(dx, dy)
                if dist[v] > (dist[u] + w):
                    dist[v] =  (dist[u] + w)
                    #add traceback parent
                    parent[v] = u

        # --- Step 5: reconstruct path ---
        rx, ry = [], []
        u = goal_id
        if parent[u] is not None or u == start_id:  # path exists
            while u is not None:
                rx.append(node_x[u])
                ry.append(node_y[u])
                u = parent[u]
            # invert since we started the list with goal
            rx.reverse()
            ry.reverse()

        # plot
        for i in range(len(rx)):
            plt.plot(rx[i], ry[i], 'or')
            plt.ioff()
            plt.draw()
            plt.pause(0.0001)
            plt.show(block=False)
        return rx, ry

    def search(self, sx, sy, gx, gy, node_x, node_y, edge_ids_list):
        """
        Search shortest path using Dijkstra's algorithm.
        This implementation Grows the open set dynamically starting with just the start node in the open set:
        more memory-efficient (only keeps nodes that are reachable).

        sx, sy: start coordinates [m]
        gx, gy: goal coordinates [m]
        node_x: list of x positions of nodes in the graph
        node_y: list of y positions of nodes in the graph
        edge_ids_list: Graph is given as   adjacency list of node indices. (for each node index, the list of connected nodes)
        """

        # Initialize start and goal nodes
        start_node = self.Node(sx, sy, 0.0, -1)  # dist = 0, no parent (last entry) for start node
        goal_node = self.Node(gx, gy, 0.0, -1)  # dist = 0, no parent (last entry) for start node

        current_node = None

        # Open set = nodes discovered but not yet visited
        # Closed set = nodes already processed
        open_set, close_set = dict(), dict()
        # This looks up the index of start_node in the roadmap, get the index and set the
        # the dictionary open_set that stores nodes waiting to be explored inserting start_node at
        #that index
        #Add the start node (with dist 0) into the open set, keyed by its roadmap index.”
        open_set[self.find_id(node_x, node_y, start_node)] = start_node

        while True:
            # Check if goal is reached, 
            if self.has_node_in_set(close_set, goal_node):
                print("goal is found!")
                goal_node.parent = current_node.parent
                goal_node.dist = current_node.dist
                #Algorithm stops when the goal node is in the closed set.
                break

            # If no nodes left to explore → path not found
            elif not open_set:
                print("Dijkstra: Cannot find path")
                sys.exit()
                break

            # --- from all the node in open set Pick node with lowest distance ---
            current_id = None
            lowest_dist = float("inf")
            for node_id in open_set:
                dist = open_set[node_id].dist
                if dist < lowest_dist:
                    lowest_dist = dist
                    current_id = node_id

            # This is the node with lower distance among the ones to be visited that
            # we will expand
            current_node = open_set[current_id]

            # Optional visualization
            if self.show_animation and len(close_set.keys()) % 2 == 0:
                plt.plot(current_node.x, current_node.y, "xg")
                plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [sys.exit(0) if event.key == 'escape' else None])
                plt.pause(0.1)

            # Move node from open → closed set
            del open_set[current_id]
            close_set[current_id] = current_node

            # --- Expand neighbors ---
            for i in range(len(edge_ids_list[current_id])):
                neighbour_node_id = edge_ids_list[current_id][i]

                # If already visited, skip
                if neighbour_node_id in close_set:
                    continue
                #get neighbour node from index
                neighbour_node_x = node_x[neighbour_node_id]
                neighbour_node_y = node_y[neighbour_node_id]

                # Distance from current node to neighbor (e.g. edge cost) with Euclidean norm
                dx =  neighbour_node_x - current_node.x
                dy = neighbour_node_y - current_node.y
                d = math.hypot(dx, dy)  # Euclidean distance

                # Create neighbour node struct with cumulated dist setting current node as its parent
                neighbour_node = self.Node(neighbour_node_x, neighbour_node_y, current_node.dist + d, current_id)

                # If already in open set, keep the lower-dist version otherwise put in open set
                if neighbour_node_id in open_set:
                    if open_set[neighbour_node_id].dist > neighbour_node.dist:
                        open_set[neighbour_node_id] = neighbour_node
                else:
                    open_set[neighbour_node_id] = neighbour_node

        #When you take a node out of open_set and put it into close_set,
        # it is the node with the minimum current distance in the entire frontier.
        #That means you’ve already found the shortest possible path to that node —
        # no future cheaper path can appear.
        #While in open_set → costs can improve. once in the visited_set dist cannot change
        # Reconstruct path from goal to start
        rx, ry = self.generate_final_path(close_set, goal_node)

        return rx, ry

    def generate_final_path(self, close_set, goal_node):
        rx, ry = [goal_node.x], [goal_node.y]
        #start from the goal node and backtrack from goasl to start through the parents
        parent = goal_node.parent
        while parent != -1:
            n = close_set[parent]
            rx.append(n.x)
            ry.append(n.y)
            parent = n.parent
        rx, ry = rx[::-1], ry[::-1]  # reverse it
        return rx, ry

    def has_node_in_set(self, target_set, node):
        for key in target_set:
            if self.is_same_node(target_set[key], node):
                return True
        return False

    def find_id(self, node_x_list, node_y_list, target_node):
        """
        Find the index (ID) of a node in the roadmap.

        node_x_list: list of x coordinates of all nodes
        node_y_list: list of y coordinates of all nodes
        target_node: the Node object we are searching for

        Returns:
            index (int) of the matching node in the lists,
            or None if no match is found.
        """

        # Loop through all node positions
        for i, _ in enumerate(node_x_list):

            # Check if the coordinates of the i-th node match the target node
            if self.is_same_node_with_xy(node_x_list[i], node_y_list[i],
                                         target_node):
                return i  # return the index if match found

        # If no match was found, return None
        return None


    def is_same_node_with_xy(self, node_x, node_y, node_b):
        dist = np.hypot(node_x - node_b.x,
                        node_y - node_b.y)
        return dist <= 0.1


    def is_same_node(self, node_a, node_b):
        dist = np.hypot(node_a.x - node_b.x,
                        node_a.y - node_b.y)
        return dist <= 0.1

    def random_sparse_graph(self, num_nodes=5, max_edges_per_node=3, max_weight=10):
        nodes = list(range(num_nodes))
        adjacency_graph = {i: [] for i in nodes}

        for node in range(num_nodes):
            # Choose random neighbors (without self-loops)
            neighbors = random.sample(
                [n for n in range(num_nodes) if n != node],
                k=random.randint(1, max_edges_per_node)
            )
            for neigh in neighbors:
                weight = random.randint(1, max_weight)
                adjacency_graph[node].append((neigh, weight))

        return nodes, adjacency_graph

    def search_map(self, start, goal, map):
        """
        Search shortest path using Dijkstra's algorithm.
        The adjacency graph is not provided and the map is explored
        start: start coordinates [m]
        goal: goal coordinates [m]
        """
        plt.imshow(map,  cmap='gray_r')  # shows the map
        plt.plot(start[1], start[0], 'or',markersize=10)  # puts a red asterisk at the start
        plt.plot(goal[1], goal[0], 'oy',markersize=10)  # puts a yellow asterisk at the goal
        plt.ion()  # turns 'interactive mode' on

        # --- Step 1: initialize distances and parents ---
        dist = defaultdict(lambda:float("inf")) # initialize all entries with infinity even if number of nodes is not known in advance
        dist[start] = 0.0  # except the start for which is zero
        parent = {start: None}  # data structure to store the id of the predecessor for each node for shortest path

        # --- Step 2: unvisited and visited list
        Q = [(0.0, start)]
        visited = set()  # set to store visited nodes, empty (with x,y coordinates using tuble enables check of membership)
        number_of_expanded_nodes = 0
        # --- Step 3: main loop ---
        while Q:  # while queue not empty
            # select the element of Q with minimum stored distance
            _, u = heappop(Q)
            # Skip if already visited
            if u in visited:
                continue
            visited.add(u)  # mark as visited
            number_of_expanded_nodes+=1

            plt.plot(u[1], u[0], 'g*')
            plt.show()
            plt.pause(0.000001)
            if u == goal:
                break
            # check neighbours
            for v, w in  get8NeighborsCost(u, map):
                if v not in visited:  # if v hass not been visited so far
                    # v = neighbour index
                    # w = edge weight
                    newcost =(dist[u] + w)
                    if  newcost < dist[v]:
                        dist[v] = (dist[u] + w)
                        heappush(Q, (newcost, v))
                        # add traceback parent
                        parent[v] = u
        # --- Step 5: reconstruct path ---
        path = self.compute_path(start, goal, parent)
        for p in path:
             plt.plot(p[1],p[0],'r.',markersize=10)
             plt.ioff()
             plt.show()
        path_lenght = dist[goal]
        return number_of_expanded_nodes, path_lenght

if __name__ == "__main__":
    print(__file__ + " start!!")

    # Class Exercise
    # start = 'A'
    # goal = 'E'
    # nodes = ['A', 'B','C','D','E']
    # #since we have letter we use a disctionary for the  weighted graph
    # edge_ids_list_w = {
    #     'A': [('B', 10), ('C', 3)], #(idx, weight)
    #     'B' : [('C',2), ('D',2)],
    #     'C' : [('E',2),  ('D',8) , ('B',4)],
    #     'D' : [('E',7)],
    #     'E' : [('D',9)]}
    # path = DijkstraSearch(show_animation=False, verbose=True).search_graph(start, nodes, edge_ids_list_w)

    # Question 1
    # start = 0
    # goal = 77
    # # create random sparse graph  with 100 nodes
    # nodes, edge_ids_list_w = DijkstraSearch(show_animation=False).random_sparse_graph(num_nodes=100, max_edges_per_node=10, max_weight=10)
    # path = DijkstraSearch(show_animation=False).search_graph(start, nodes, edge_ids_list_w, goal)


    # Question 2 Compare runtimes with a naive O(|V|^2) array-based implementation
    # graph_sizes = [50, 100, 200, 400, 800]
    # start = 0
    # runtimes=[]
    # for num_nodes in graph_sizes:
    #     nodes, edge_ids_list_w = DijkstraSearch(show_animation=False).random_sparse_graph(num_nodes=num_nodes, max_edges_per_node=40, max_weight=10)
    #     t0 = time.time()
    #     path = DijkstraSearch(show_animation=False).search_graph(start, nodes, edge_ids_list_w, use_heap = True)
    #     t1 = time.time()
    #     runtimes.append(t1 - t0)
    #     print(f"V={num_nodes:3d}, runtime={t1 - t0:.6f} sec")
    # plt.figure()
    # plt.grid()
    # plt.plot(graph_sizes, runtimes, "o-", label="Measured runtime")
    # scale = runtimes[0] / (graph_sizes[0] ** 2)
    # plt.plot(graph_sizes, [scale * (V ** 2) for V in graph_sizes], "--", label="Quadratic trend (V^2)")
    # plt.xlabel("Number of vertices V")
    # plt.ylabel("Runtime (s)")
    # plt.ylim([0,0.02])
    # plt.title("Dijkstra runtime growth (list-based implementation)")
    # plt.show()

    #Question 3
    np.random.seed(0)
    rows = 20
    cols = 20
    map = np.random.rand(rows, cols) < 0.1  # when the value is lower than 0.1 is True and is an obstacle, otherwise is zero (free space)
    start = (0, 0)
    goal = (19, 19)
    t0 = time.time()
    number_of_expanded_nodes, path_lenght = DijkstraSearch(show_animation=False).search_map(start, goal, map)
    t1 = time.time()
    print(f"n. of exp. nodes={number_of_expanded_nodes}, path length {path_lenght}, runtime={t1 - t0:.6f} sec")