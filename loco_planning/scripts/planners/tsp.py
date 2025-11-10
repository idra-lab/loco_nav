import numpy as np
import pulp as pl
from planners.prm import PRM
import math
import itertools
import matplotlib.pyplot as plt
import heapq

def euclid(a, b):
    """Euclidean distance between two (x, y) points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])

def shortest_path(src, dst, V, E):
    dist, prev = dijkstra(src, V, E)
    if math.isinf(dist[dst]):
        return [], float('inf')
    path = []
    cur = dst
    while cur is not None:
        path.append(cur)
        if cur == src: break
        cur = prev[cur]
    return path[::-1], dist[dst]


def dijkstra(source, V, E):
    dist = {v: float('inf') for v in V}
    parent = {v: None for v in V}
    dist[source] = 0.0
    pq = [(0.0, source)]
    while pq:
        dcur, u = heapq.heappop(pq)
        if dcur > dist[u]:
            continue
        for v in E.get(u, []):
            w = euclid(u, v)
            nd = dcur + w
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, parent

def solve_tsp_pulp(V, E, cities, start):
    """
    Traveling Salesman Problem (TSP)
    MTZ formulation with full constraints.
    Each city is visited exactly once and tour returns to start.
    """

    import math
    import pulp as pl

    key_nodes = list(cities)
    N = len(key_nodes)

    # ---- Compute shortest path distances ----
    sp_dist = {}
    for n in key_nodes:
        dist_from_n, _ = dijkstra(n, V, E)
        sp_dist[n] = dist_from_n

    dist = {}
    for i in key_nodes:
        for j in key_nodes:
            if i == j:
                continue
            d = sp_dist[i][j]
            if not math.isinf(d):
                dist[(i, j)] = d


    # ---- MILP model ----
    model = pl.LpProblem("TSP_MTZ", pl.LpMinimize)

    # Binary edge variables
    x = pl.LpVariable.dicts("x", dist.keys(), 0, 1, cat="Binary")

    # MTZ u_i order variables,   u_start fixed to 1; others in [2, N]
    u = {}
    for n in key_nodes:
        if n == start:
            u[n] = pl.LpVariable(f"u_{n}", lowBound=1, upBound=1, cat="Continuous")
        else:
            u[n] = pl.LpVariable(f"u_{n}", lowBound=2, upBound=N, cat="Continuous")

    # ---- Objective ----
    objective_terms = []
    for (i, j) in dist.keys():
        if i == j:
            continue
        objective_terms.append(dist[(i, j)] * x[(i, j)])
    model += pl.lpSum(objective_terms)

    # ---- Constraints ----
    # (1) Each node has exactly one outgoing arc
    for i in key_nodes:
        outgoing = []
        for j in key_nodes:
            if i == j:
                continue
            if (i, j) in x:
                outgoing.append(x[(i, j)])
        model += pl.lpSum(outgoing) == 1, f"out_degree_{i}"

    # (2) Each node has exactly one incoming arc
    for j in key_nodes:
        incoming = []
        for i in key_nodes:
            if i == j:
                continue
            if (i, j) in x:
                incoming.append(x[(i, j)])
        model += pl.lpSum(incoming) == 1, f"in_degree_{j}"

    # (3) MTZ subtour elimination: apply only for nodes 2..N (skip start, skip i==j, skip arcs touching start)
    # u[i] - u[j] + N*x[i,j] <= N-1  for all i!=j, i!=start, j!=start with (i,j) in key_nodes
    for i in key_nodes:
        for j in key_nodes:
            if i == start or j == start or i == j:
                continue
            if (i, j) in x:
                model += u[i] - u[j] + N * x[(i, j)] <= N - 1, f"mtz_{i}_{j}"

    # ---- Solve ----
    model.solve(pl.PULP_CBC_CMD(msg=False))
    print("Solver status:", pl.LpStatus[model.status])
    if pl.LpStatus[model.status] != "Optimal":
        raise RuntimeError(f"TSP solve did not find an optimal tour (status={pl.LpStatus[model.status]}).")

    # ---- Extract results ----
    arcs_chosen = []
    for (i, j) in x:
        if pl.value(x[(i, j)]) > 0.5:
            arcs_chosen.append((i, j))

    total_length = 0.0
    for (i, j) in arcs_chosen:
        total_length += dist[(i, j)]

    # ---- Reconstruct ordered tour ----
    succ = {}
    for (i, j) in arcs_chosen:
        succ[i] = j

    tour = [start]
    current = start
    visited = {start}
    while True:
        current = succ[current]
        tour.append(current)
        if current == start:
            break
        if current in visited:
            break
        visited.add(current)

    # ---- Expand full PRM path ----
    full_path = []
    for a, b in zip(tour[:-1], tour[1:]):
        seg, _ = shortest_path(a, b, V, E)
        if not full_path:
            full_path.extend(seg)
        else:
            full_path.extend(seg[1:])

    best = {
        "best_tour": tour,
        "best_length": total_length,
    }

    return best, full_path


def visualization_tsp(V, cities, start, best, full_path):
    """
    Visualization of the TSP tour and roadmap.
    - Cities are green
    - Start/End is gray square
    - Tour arcs (between cities) in thick red
    - Expanded PRM path (full_path) in thin red
    """
    # Extract tour from best result
    tour = best.get("best_tour", [])
    order_map = {city: idx for idx, city in enumerate(tour[:-1])}  # exclude duplicate last=start

    # Draw tour connections (as polylines) between cities (in straight lines)
    if tour:
        for a, b in zip(tour[:-1], tour[1:]):
            plt.plot([a[0], b[0]], [a[1], b[1]], color="red", linewidth=3, alpha=0.3)

    # Draw the expanded optimal PRM path on tour
    if full_path:
        for a, b in zip(full_path[:-1], full_path[1:]):
            plt.plot([a[0], b[0]], [a[1], b[1]], color="red", linewidth=2.0)

    # Plot the city nodes (green)
    for node in cities:
        plt.scatter(node[0], node[1], color="limegreen", s=90, edgecolor="black", zorder=5)
        if node in order_map:
            order = order_map[node]
            plt.text(node[0] + 0.2, node[1] + 0.2, f"{order}", fontsize=9, fontweight="bold", color="blue")

    # Highlight the start node
    plt.scatter(start[0], start[1], color="gray", marker="s", s=110, edgecolor="black", zorder=6)
    plt.text(start[0] + 0.2, start[1] + 0.2, "start/end", fontsize=10, color="black")

    plt.title(f"TSP Optimal Tour (red)\nLength = {best['best_length']:.2f} m")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.xlabel("X [m]")
    plt.xlim([0,10])
    plt.ylim([0,10])
    plt.ylabel("Y [m]")
    plt.axis("equal")
    plt.show()



def snap_to_nearest_node(input_nodes, V):
    """
    Snap each node position to the nearest roadmap node.

    Parameters
    ----------
    victims : list of (x, y)
        The actual (possibly off-node) victim coordinates.
    V : list of (x, y)
        The roadmap node coordinates.

    Returns
    -------
    dict
        Mapping from original victim coordinate -> nearest node in V
    """
    def euclid(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    snapped = []

    if isinstance(input_nodes, tuple):
        snapped = min(V, key=lambda n: euclid(input_nodes, n))
    else:
        for v in input_nodes:
            nearest = min(V, key=lambda n: euclid(v, n))
            snapped.append(nearest)
    return snapped



def set_obstacle(center, size, resolution, map):
    # Convert from world coordinates (meters) to map indices
    ry = int(center[1] / resolution)
    cx = int(center[0] / resolution)
    half_size = int((size / 2) / resolution)
    # Define square boundaries (clip to map)
    row_min = max(ry - half_size, 0)
    row_max = min(ry + half_size, map.shape[0] - 1)
    col_min = max(cx - half_size, 0)
    col_max = min(cx + half_size, map.shape[1] - 1)
    # Fill the region
    map[row_min:row_max, col_min:col_max] = 1
    return map




if __name__ == '__main__':
    map_width = 10.
    map_height = 10.
    resolution = 0.1
    map = np.zeros((int(map_width / resolution),int(map_height / resolution) ))

    #square  centered at(2, 8) size = 2
    set_obstacle(center=(2,8), size=2, resolution=resolution, map=map)
    # square  centered at(4,4.5) size = 1.5
    set_obstacle(center=(4,4.5), size=1.5, resolution=resolution, map=map)
    # square  centered at(4,4.5) size = 1
    set_obstacle(center=(8, 4), size=1, resolution=resolution, map=map)
    prm = PRM(n_samples=300, k=10, resolution=resolution, seed=0)

    # 1. Build roadmap V = Vertex, E = Edges
    V, E = prm.construct_roadmap(map)

    # TSP params (visit all these points and return to start)
    cities_real = [(4, 9),  (8, 9),(1.5, 2.2) , (6.7, 5.4),  (3, 6.34),(9.1, 9.5) ]
    cities = snap_to_nearest_node(cities_real, V)
    start = cities[0]  # or pick any other in 'cities'

    best_result, full_path = solve_tsp_pulp(V, E, cities, start)



    visualization_tsp(V, cities, start, best_result, full_path)
    print(best_result)