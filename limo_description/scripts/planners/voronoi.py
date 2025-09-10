"""
Voronoi Road Map Planner (commented)


Overview
--------
This module demonstrates a *Voronoi Roadmap (VRM)* approach for 2D path planning.
It:
  1) Generates Voronoi vertices from obstacle points (these lie maximally far from obstacles),
  2) Connects nearby vertices with collision-free edges (roadmap),
  3) Uses Dijkstra's shortest path on the roadmap to connect start → goal.

Dependencies
------------
- numpy, matplotlib
- scipy.spatial (Voronoi, cKDTree)
- a local Dijkstra implementation at VoronoiRoadMap/dijkstra_search.py
"""

from __future__ import annotations
import matplotlib
matplotlib.use('TkAgg')
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree, Voronoi
import sys
import pathlib

# Allow importing sibling package (for DijkstraSearch)
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from planners.dijkstra_search import DijkstraSearch

# Toggle to visualize steps
show_animation = True


class VoronoiRoadMapPlanner:
    """Planner that builds a Voronoi-based roadmap and searches it with Dijkstra.

    Attributes
    ----------
    N_KNN : int
        Max number of neighbor connections per roadmap node.
    MAX_EDGE_LEN : float
        Hard cap on edge length (meters) to avoid overly long connections that
        may cut through clutter or hurt numerical stability.
    """

    def __init__(self, start, goal, obstacle_list, robot_radius) -> None:
        # Tuning parameters
        self.N_KNN = 10           # number of neighbors to try per sampled point
        self.MAX_EDGE_LEN = 30.0  # [m] maximum edge length allowed
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacle_list
        self.robot_radius = robot_radius

    def planning(self,  show_animation=True):

        """Plan a path from (sx, sy) to (gx, gy).


        Parameters
        ----------
        sx, sy : float
            Start coordinates.
        gx, gy : float
            Goal coordinates.
        obstacle_list : list[float]
            Obstacle point clouds (x and y lists).
        robot_radius : float
            Circular robot radius used for conservative collision checks.

        Returns
        -------
        (rx, ry) : tuple[list[float], list[float]]
            Path coordinates if found. Empty lists if no path.
        """
        sx = self.start[0]
        sy = self.start[1]
        gx = self.goal[0]
        gy = self.goal[1]
        ox = [p[0] for p in self.obstacle_list ]
        oy = [p[1] for p in self.obstacle_list ]



        # KD-tree accelerates nearest-obstacle distance queries used in collision checks
        obstacle_tree = cKDTree(np.vstack((ox, oy)).T)

        # 1) Sample candidate nodes from Voronoi diagram of obstacles (+ start/goal)
        sample_x, sample_y = self.voronoi_sampling(sx, sy, gx, gy, ox, oy)
        if show_animation:  # pragma: no cover (skip in tests)
            plt.figure()
            plt.plot(ox, oy, ".k", label="obstacles")
            plt.plot(sx, sy, "ob", markersize=10, label="start")
            plt.plot(gx, gy, "or", markersize=10, label="goal")
            plt.grid(True)
            plt.plot(sample_x, sample_y, ".g", label="Voronoi vertices")
            plt.axis("equal")
            plt.legend(loc="best")
        # 2) Build roadmap edges between nodes (subject to collision-free straight lines)
        road_map_info = self.generate_road_map_info(
            sample_x, sample_y, self.robot_radius, obstacle_tree
        )

        # 3) Run Dijkstra over the roadmap (DijkstraSearch accepts the samples and adjacency)
        rx, ry = DijkstraSearch(show_animation).search(
            sx, sy, gx, gy, sample_x, sample_y, road_map_info
        )
        path = np.column_stack((rx, ry))
        return path

    def is_collision(
        self,
        sx: float,
        sy: float,
        gx: float,
        gy: float,
        rr: float,
        obstacle_kd_tree: cKDTree,
    ) -> bool:
        """Check if the straight segment (sx, sy) → (gx, gy) collides with obstacles.

        Strategy: march along the segment in steps of size D=rr (robot radius),
        query nearest obstacle distance from KD-tree, and declare collision if
        any point (or the endpoint) is within radius rr.
        Also reject edges longer than MAX_EDGE_LEN to keep the graph local.
        """
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        yaw = math.atan2(dy, dx)
        d = math.hypot(dx, dy)

        # Avoid very long edges that may skip around obstacles or bloat the graph
        if d >= self.MAX_EDGE_LEN:
            return True

        D = rr               # step size along the edge (conservative)
        n_step = round(d / D)

        # March and check clearance
        for _ in range(n_step):
            dist, _ = obstacle_kd_tree.query([x, y])
            if dist <= rr:
                return True  # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        # Also ensure the goal endpoint itself is safe
        dist, _ = obstacle_kd_tree.query([gx, gy])
        if dist <= rr:
            return True

        return False  # edge is collision-free

    def generate_road_map_info(
        self,
        node_x: list[float],
        node_y: list[float],
        rr: float,
        obstacle_tree: cKDTree,
    ) -> list[list[int]]:
        """Construct adjacency list for the roadmap.

        For each node, query its neighbors by distance and try to connect up to
        N_KNN edges via straight-line collision checks.

        Returns
        -------
        road_map : list[list[int]]
            road_map[i] is the list of neighbor indices for node i.
        """
        road_map: list[list[int]] = []
        n_sample = len(node_x)

        # KD-tree over nodes for fast KNN queries
        node_tree = cKDTree(np.vstack((node_x, node_y)).T)

        # For each node, walk outward to nearest others and attempt connections
        for (i, ix, iy) in zip(range(n_sample), node_x, node_y):
            # Query *all* neighbors ordered by distance; we'll stop early at N_KNN
            dists, indexes = node_tree.query([ix, iy], k=n_sample)

            edge_id: list[int] = []

            # indexes[0] == i (self); start from 1 to skip self
            for ii in range(1, len(indexes)):
                nx = node_x[indexes[ii]]
                ny = node_y[indexes[ii]]

                # Only accept edge if collision-free
                if not self.is_collision(ix, iy, nx, ny, rr, obstacle_tree):
                    edge_id.append(indexes[ii])

                # Cap number of neighbors per node for sparsity
                if len(edge_id) >= self.N_KNN:
                    break

            road_map.append(edge_id)

        # If desired, you can visualize with plot_road_map(road_map, node_x, node_y)
        return road_map

    @staticmethod
    def plot_road_map(road_map: list[list[int]], sample_x: list[float], sample_y: list[float]) -> None:  # pragma: no cover
        """Utility to plot the roadmap edges for debugging/visualization."""
        for i, _ in enumerate(road_map):
            for ii in range(len(road_map[i])):
                ind = road_map[i][ii]
                plt.plot([sample_x[i], sample_x[ind]], [sample_y[i], sample_y[ind]], "-k")

    @staticmethod
    def voronoi_sampling(
        sx: float,
        sy: float,
        gx: float,
        gy: float,
        ox: list[float],
        oy: list[float],
    ) -> tuple[list[float], list[float]]:
        """Sample roadmap nodes from Voronoi vertices of obstacle points + start/goal.

        The Voronoi vertices tend to lie in regions equidistant from obstacles,
        which are intuitively good corridors for safe navigation.
        """
        # Build obstacle point array shape (N, 2)
        oxy = np.vstack((ox, oy)).T

        # Compute Voronoi diagram & collect its vertices as samples
        vor = Voronoi(oxy)
        sample_x = [vx for vx, _ in vor.vertices]
        sample_y = [vy for _, vy in vor.vertices]

        # Always include start and goal as graph nodes
        sample_x.append(sx)
        sample_y.append(sy)
        sample_x.append(gx)
        sample_y.append(gy)

        return sample_x, sample_y


if __name__ == "__main__":
    print(__file__ + " start!!")

    # Start and goal
    start = np.array([10.0, 10.0, 0.])  # [m]
    goal = np.array([50.0, 50.0, 0.]) # [m]
    obstacle_list = []
    robot_size = 5.0  # [m] (used as the collision-check radius)

    # Construct a toy map with axis-aligned walls and two internal walls
    ox: list[float] = []
    oy: list[float] = []

    # Outer rectangle boundary
    for i in range(60):
        obstacle_list.append([float(i), 0.0])
    for i in range(60):
        obstacle_list.append([60.0, float(i)])
    for i in range(61):
        obstacle_list.append([float(i), 60.0])
    for i in range(61):
        obstacle_list.append([0.0, float(i)])

    # Internal vertical wall (left)
    for i in range(40):
        obstacle_list.append([20.0, float(i)])

    # Internal diagonal wall (right)
    for i in range(40):
        obstacle_list.append([40.0, 60.0 - i])

    voronoi_planner = VoronoiRoadMapPlanner(start, goal, obstacle_list,  robot_size)

    # Plan
    path = voronoi_planner.planning(show_animation = show_animation)

    assert path.size > 0, "Cannot find path"

    if show_animation:  # pragma: no cover
        plt.plot(path[:,0], path[:,1], "-r", label="path")
        plt.pause(0.1)
        plt.show()



