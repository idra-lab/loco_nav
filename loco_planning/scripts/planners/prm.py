"""
Probabilistic Roadmap (PRM)
Path Planning Sample Code

Inspired by: PythonRobotics and PRM pseudocode
"""

import math
import random
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import time
from math import sqrt, atan2, sin, cos
from dijkstra_search import DijkstraSearch

class PRM:
    def __init__(self, n_samples=200, k=10, resolution=0.5, subcell_sampling_factor=0.25, seed=None):
        self.n_samples = n_samples          # number of random samples
        self.k = k                          # number of nearest neighbors
        self.resolution = resolution        # grid resolution
        self.subcell_sampling_factor = subcell_sampling_factor
        if seed is not None:
            random.seed(seed)

    def sample_free(self, map, width, height):
        """Return a random collision-free sample (q ∈ C_free)."""
        while True:
            q = (random.uniform(0, height), random.uniform(0, width))
            ry, cx = int(q[1] / self.resolution), int(q[0] / self.resolution)
            if not map[ry, cx]:
                return q

    def distance(self, p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def is_collision_free(self, q_nearest, q_new, map):
        """Check if the line between q_nearest and q_new crosses an obstacle."""
        rows, cols = map.shape

        # get the number of subsample for collision checking on the edge between q_nearest and q_rand
        n = int(self.distance(q_nearest, q_new) / (self.resolution * self.subcell_sampling_factor))
        if n == 0:  # the point is inside the ball of radius of the resolution so check the point right away
            ry, cx = int(q_new[1] / self.resolution), int(q_new[0] / self.resolution)
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
                ry, cx = int(y / self.resolution), int(x / self.resolution)
                # discard points outside the map
                if ry < 0 or cx < 0 or ry >= rows or cx >= cols:
                    return False
                # discard points on obstacles (positive map)
                if map[ry, cx]:
                    return False
        return True

    def find_k_nearest(self, nodes, q, k):
        """Return indices of k-nearest nodes to q."""
        dlist = [self.distance(q, n) for n in nodes]
        sorted_idx = np.argsort(dlist)
        return [nodes[i] for i in sorted_idx[:k]]

    def construct_roadmap(self, map):
        rows, cols = map.shape
        width = cols * self.resolution
        height = rows * self.resolution
        plt.imshow(map, cmap='gray_r', origin='lower', extent=[0, width, 0, height])

        """Main PRM roadmap construction."""
        V = []     # vertices
        E = {}     # adjacency list (edges)

        # 1. Sample n free configurations
        for i in range(self.n_samples):
            q = self.sample_free(map, width, height)
            V.append(q)
            E[q] = []
        # 2. For each vertex, find K-nearest and connect
        for q in V:
            q_near = self.find_k_nearest(V, q, self.k)
            for q_prime in q_near:
                if q == q_prime or q_prime in E[q]:
                    continue
                if self.is_collision_free(q, q_prime, map):
                    E[q].append(q_prime)
                    E[q_prime].append(q)
        #plot edges and vertices
        for q in V:
            plt.scatter( q[0], q[1],  color="black", s=10, zorder=4, alpha=0.5)
            for qn in E[q]:
                plt.plot( [q[0], qn[0]],[q[1], qn[1]], "-g", linewidth=0.5, alpha=0.5)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.pause(0.001)

        return V, E

    def dijkstra(self, E, start, goal):
        """Simple Dijkstra shortest path on PRM graph."""
        import heapq
        Q = []
        heapq.heappush(Q, (0, start))
        dist = {start: 0}
        parent = {start: None}
        while Q:
            cost, u = heapq.heappop(Q)
            if u == goal:
                break
            for v in E[u]:
                new_cost = cost + self.distance(u, v)
                if v not in dist or new_cost < dist[v]:
                    dist[v] = new_cost
                    parent[v] = u
                    heapq.heappush(Q, (new_cost, v))
        if goal not in parent:
            return None
        # reconstruct path
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parent[node]
        return path[::-1]

    def find_path(self, start, goal, map):

        # 1. Build roadmap V = Vertex, E = Edges
        V, E = self.construct_roadmap(map)

        plt.plot(start[0], start[1],  'or', markersize=10)
        plt.plot(goal[0], goal[1],  'oy', markersize=10)

        # 2. Add start and goal to roadmap
        V.extend([start, goal])
        E[start] = []
        E[goal] = []

        for q in self.find_k_nearest(V, start, self.k):
            if self.is_collision_free(start, q, map):
                E[start].append(q)
                E[q].append(start)
        for q in self.find_k_nearest(V, goal, self.k):
            if self.is_collision_free(goal, q, map):
                E[goal].append(q)
                E[q].append(goal)

        # 3. Find path via Dijkstra (differently from search static, the E is a dictionary {(x,y) : [(x,y), (x,y)...] with adjacency list for all nodes)
        path = self.dijkstra(E, start, goal)

        # 4. Plot result
        if path:
            for p1, p2 in zip(path[:-1], path[1:]):
                plt.plot( [p1[0], p2[0]],[p1[1], p2[1]], 'r-', linewidth=3)
            plt.plot([p[0] for p in path], [p[1] for p in path], 'ro', markersize=4)
            plt.show()
            length = sum(self.distance(p1, p2) for p1, p2 in zip(path[:-1], path[1:]))
            return path, length
        else:
            print("No path found.")
            plt.show()
            return None, None


if __name__ == '__main__':
    np.random.seed(0)
    start = (1., 1.)
    goal = (9., 9.)
    map_width = 10.
    map_height = 10.
    resolution = 0.5
    map = np.random.rand(int(map_width / resolution), int(map_height / resolution)) < 0.1
    prm = PRM(n_samples=400, k=10, resolution=resolution)
    t0 = time.time()
    path, length = prm.find_path(start, goal, map)
    t1 = time.time()
    if path:
        print(f"Path found! Length = {length:.3f}, runtime = {t1 - t0:.3f} sec")
