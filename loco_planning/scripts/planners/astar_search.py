"""

Dijkstra Search library

author: Atsushi Sakai (@Atsushi_twi)

"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
from collections import defaultdict
from heapq import heappush, heappop
from utils.utils import get8NeighborsCost


class AstarSearch:


    def __init__(self, verbose=False):
        self.verbose = verbose

    def search_graph(self, start, goal, nodes, edge_ids_list_g, h):

        """
            Simplified Astar's algorithm. static method (all nodes in open set at ∞) is simpler mathematically, but wastes space if the graph is huge and sparse.
            start node
            graph
            edge_ids_list : adjacency list (list of neighbors for each node index) with weight for each edge
            heuristics: function that implements heuristics
            
        """
        open_set = [start] # set of nodes that have not been explored yet
        closed_set = [] # set of nodes that cannot be expanded anymore

        # --- Step 2: initialize distances and parents ---
        g = {node: math.inf for node in nodes} # set all nodes distances to infinity
        g[start] = 0.0  # except the start for which is zero
        parent = {node: None for node in nodes}  # data structure to store the id of the predecessor for each node for shortest path

        iter = 0
        while open_set: # continue till the list is empty
            iter+=1
            #The heuristic is only used when deciding which node to expand next, When pick the next node from open_list choose the one with lowest estimated total cost
            n = self.mincost(open_set, g, h)
            if n == goal: #reconstruct and return the path.
                # reconstruct path
                path = []
                while n is not None:
                    path.append(n)
                    n = parent[n]
                # invert since we started the list with goal
                path.reverse()
                if self.verbose:
                    print("Astar converged")
                #print("parent", parent)
                path_lenght = g[goal]
                return path, path_lenght
            open_set.remove(n)

            # expand n for all the sucessors
            for v, cost_nv in edge_ids_list_g[n]:
                # Skip neighbors already in Closed because already explored
                if v not in closed_set:
                    # Compute tentative cost for new path connecting v to n
                    tentative_cost = g[n] + cost_nv
                    #  if new cost is better → update.
                    if tentative_cost < g[v]:
                        g[v] = tentative_cost
                        parent[v] = n
                        if v not in open_set:
                            open_set.append(v)
            #once all successors have been expanded put node in closed
            closed_set.append(n)
            if self.verbose:
                input(f"\niter: {iter}, Press Enter to continue...")
                print(f" Expanding: {n}")
                print(" Closed:", closed_set)
                print(" Open:", [(node, g[node], g[node] + h[node]) for node in open_set])

        return None, float("inf")  # no path found

    def mincost(self, open_set, g, h):
        # Start by assuming the first element of Q is the best
        u = open_set[0]
        min_total_cost = g[u] + h[u]
        # Loop through all other nodes in Q
        for candidate in open_set[1:]:
            total_cost = g[candidate] + h[candidate]
            if total_cost  < min_total_cost:
                min_total_cost = total_cost
                u = candidate
        return u

    def search_map(self, start, goal, map, h):
        """
        Search shortest path using Dijkstra's algorithm.
        The adjacency graph is not provided and the map is explored
        start: start coordinates [m]
        goal: goal coordinates [m]
        """
        plt.imshow(map)  # shows the map
        plt.plot(start[1], start[0], 'or')  # puts a red asterisk at the start
        plt.plot(goal[1], goal[0], 'oy')  # puts a yellow asterisk at the goal
        plt.ion()  # turns 'interactive mode' on

        # --- Step 1: initialize distances and parents ---
        open_set = [(0 + h(start, goal), start)]  # (f, node)  # set of nodes that have not been explored yet
        closed_set = set()  # set of nodes that cannot be expanded anymore
        g = defaultdict(lambda:float("inf")) # initialize all entries with infinity even if number of nodes is not known in advance
        g[start] = 0.0  # except the start for which is zero
        parent = {start: None}  # data structure to store the id of the predecessor for each node for shortest path

        number_of_expanded_nodes = 0

        # --- Step 3: main loop ---
        while open_set: # continue till the set is empty
            # select the element of Q with minimum stored distance
            f, n = heappop(open_set)
            if n == goal: #reconstruct and return the path.
                # reconstruct path
                path = []
                while n is not None:
                    path.append(n)
                    n = parent[n]
                # invert since we started the list with goal
                path.reverse()
                print("Astar converged")
                #print("parent", parent)
                f = []
                for p in path:
                    plt.plot(p[1], p[0], 'r.')
                    plt.ioff()
                    plt.show()
                    f.append(g[p] + h(p, goal))
                path_lenght = g[goal]
                return   number_of_expanded_nodes, path_lenght, f

            plt.plot(n[1], n[0], 'g*')
            plt.show()
            plt.pause(0.000001)

            # expand n for all the sucessors
            for v, cost_nv in get8NeighborsCost(n, map):
                # Skip neighbors already in Closed because already explored
                if v not in closed_set:
                    # Compute tentative cost for new path connecting v to n
                    tentative_g = g[n] + cost_nv
                    #  if new cost is better → update.
                    if tentative_g < g[v]:
                        g[v] = tentative_g
                        parent[v] = n
                        f_v = tentative_g + h(v, goal)
                        if v not in open_set:
                            heappush(open_set, (f_v, v))

            # once all successors have been expanded put node in closed
            closed_set.add(n)
            number_of_expanded_nodes += 1
        return None

def euclidean(node, goal):
    return math.hypot(goal[0] - node[0], goal[1] - node[1])

def manhattan(node, goal):
    return abs(goal[0] - node[0]) + abs(goal[1] - node[1])

if __name__ == "__main__":
    print(__file__ + " Astar start!!")

    # Class exercise
    start = 'S'
    goal = 'G'
    nodes = ['S', 'A', 'B','C','D','E', 'F', 'G']
    #since we have letter we use a disctionary for the  weighted graph
    edge_ids_list_g = {
        'S': [('A', 2), ('D', 5)], #(idx, weight)
        'A' : [('D',2), ('B',1)],
        'B' :  [('E',5), ('C',4)],
        'D' : [('E',2)],
        'E' : [('B',2), ('F',4)],
        'C' : [],
        'F' : [('G',3)],
        'G' : []}
    heuristics = {
        'S': 11,  # (idx, weight)
        'A': 10.4,
        'B': 6.7,
        'C': 4,
        'D': 8.9,
        'E': 6.9,
        'F': 3,
        'G': 0}
    path, cost = AstarSearch(verbose=True).search_graph(start,goal, nodes, edge_ids_list_g, heuristics)
    print(f"path from {start} to {goal} is: {path} with cost {cost}")

    # question 1
    # print("\n\nSingle source")
    # for node in nodes:
    #     if node != start:
    #         path, cost = AstarSearch(verbose=False).search_graph(start, node, nodes, edge_ids_list_g, heuristics)
    #         print(f"path from {start} to {node} is: {path} with cost {cost}")

    # question 2
    # print("\n\nSingle destination") #shortest paths to one target
    # min_cost = math.inf
    # for node in nodes:
    #     if node != goal:
    #         path, cost = AstarSearch(verbose=False).search_graph(node, goal, nodes, edge_ids_list_g, heuristics)
    #         if cost<min_cost:
    #             min_cost = cost
    #             min_path = path
    # print(f"shortest path to {goal} is: {min_path} with cost {min_cost}")

    # question 3 map
    # np.random.seed(0)
    # rows = 20
    # cols = 20
    # map = np.random.rand(rows, cols) < 0.1  # when the value is lower than 0.1 is True and is an obstacle, otherwise is zero (free space)
    # start = (0, 0)
    # goal = (19, 19)
    # t0 = time.time()
    # number_of_expanded_nodes, path_lenght, f = AstarSearch().search_map(start, goal, map, euclidean)
    # t1 = time.time()
    # print(f"n. of exp. nodes={number_of_expanded_nodes}, path length {path_lenght}, runtime={t1 - t0:.6f} sec")

    #question 4: prove heuristic is admissible (never overestimates) and consistent (monotone) -> f(n) non decreasing
    # plt.figure()
    # plt.plot(f, '-or')
    # plt.grid()
    # plt.show()