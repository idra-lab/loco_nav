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

def solve_orienteering_brute_force(V, E, victims, scores, start, L):
    """
    Solve a small Orienteering Problem on a given roadmap.

    Parameters
    ----------
    V : list of tuple(float,float)
        List of node coordinates.
    E : dict
        Adjacency list mapping each node (tuple) -> list of neighbor nodes (tuples).
    victims : list of tuple(float,float)
        Subset of V representing victim nodes.
    scores : dict
        Mapping victim node -> score value.
    start : tuple(float,float)
        Starting node coordinate (must be in V).
    L : float
        Maximum travel distance (budget).

    Returns
    -------
    dict with keys:
        - "best_tour" : list of node coordinates
        - "best_score" : total collected score
        - "best_length" : path length
    """

    # Precompute shortest-path distances between  between all key nodes
    key_nodes = [start] + victims
    sp_dist = {n:  dijkstra(n, V, E)[0]  for n in key_nodes} #[0] is the distance

    # Brute-force small number of victims
    #initialize
    best = {"best_score": -1, "best_tour": None, "best_length": None}
    # Try all victim visit permutations (no return)
    for k in range(0, len(victims) + 1):
        for subset in itertools.combinations(victims, k):
            for perm in itertools.permutations(subset):
                seq = [start] + list(perm)
                length = 0.0
                feasible = True
                # pythonic way of summing lengths between concecutives nodes
                for a, b in zip(seq[:-1], seq[1:]):
                    d = sp_dist[a][b]
                    if math.isinf(d):
                        feasible = False
                        break
                    length += d
                if not feasible:
                    continue
                score = sum(scores[v] for v in subset)
                if length <= L and score > best["best_score"]:
                    best = {"best_score": score, "best_tour": seq, "best_length": length, "end": seq[-1]}

    # Reconstruct full local path
    full_path = []
    if best["best_tour"]:
        for a, b in zip(best["best_tour"][:-1], best["best_tour"][1:]):
            seg, _ = shortest_path(a, b, V, E)
            if not full_path:
                full_path.extend(seg)
            else:
                full_path.extend(seg[1:])

    return best, full_path

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

def visualization(V, start, victims, best, full_path):
    # -----------------------------
    # Visualization (assumes prm has been plotted)
    # -----------------------------

    # Draw optimal tour in red
    if best["best_tour"]:
        for a, b in zip(best["best_tour"][:-1], best["best_tour"][1:]):
            plt.plot([a[0], b[0]], [a[1], b[1]],  color="red", linewidth=3, alpha=0.4)

    # Draw optimal full_path
    for a, b in zip(full_path[:-1], full_path[1:]):
        plt.plot([a[0], b[0]], [a[1], b[1]], color="red", linewidth=2.5)

    # Plot start and victims
    for node in V:
        if node == start:
            plt.scatter( node[0],node[1], color="gray", marker="s", s=100, edgecolor="black", zorder=5)
        elif node in victims:
            plt.scatter( node[0],node[1], color="limegreen", s=90, edgecolor="black", zorder=5)
            plt.text(node[0] + 0.2, node[1] + 0.2, f"{scores[node]}", fontsize=10)
    plt.title(f"Optimal Tour (red)\nScore={best['best_score']}, Length={best['best_length']:.2f} m")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.show()



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


def solve_orienteering_pulp(V, E, victims, scores, start, L):
    """
    Solve the orienteering problem (no return to start) using PuLP MILP.

    Parameters
    ----------
    V : list of tuple(float,float)
    E : dict mapping each node -> list of neighbor nodes
    victims : list of tuple(float,float)
    scores : dict mapping victim node -> score value
    start : tuple(float,float)
    L : float distance budget

    Returns
    -------
    best : dict with 'best_score', 'best_tour', 'best_length'
    full_path : list of coordinates in the optimal tour
    """
    import pulp as pl
    import math

    # All candidate nodes = start + victims
    key_nodes = [start] + victims
    n = len(key_nodes)
    index = {node: i for i, node in enumerate(key_nodes)}

    # Build full directed graph of reachable arcs
    arcs = []
    dist = {}
    for i in key_nodes:
        for j in key_nodes:
            if i == j:
                continue
            # compute shortest-path distance in roadmap
            d_ij, _ = dijkstra(i, V, E)
            d = d_ij[j]
            if not math.isinf(d):
                arcs.append((i, j))
                dist[(i, j)] = d

    # Initialize MILP
    model = pl.LpProblem("Orienteering_MILP", pl.LpMaximize)

    # Variables
    x = pl.LpVariable.dicts("x", arcs, cat="Binary")
    y = pl.LpVariable.dicts("y", key_nodes, 0, 1, cat="Binary")
    u = pl.LpVariable.dicts("u", key_nodes, lowBound=0, upBound=n, cat="Continuous")

    # Objective: maximize collected scores
    model += pl.lpSum(scores.get(i, 0) * y[i] for i in key_nodes)

    # Degree constraints
    for i in key_nodes:
        out_i = pl.lpSum(x[(i, j)] for (i2, j) in arcs if i2 == i)
        in_i  = pl.lpSum(x[(j, i)] for (j, i2) in arcs if i2 == i)

        if i == start:
            model += out_i == 1    # must leave start
            model += in_i == 0     # cannot enter start
        else:
            model += out_i <= y[i]
            model += in_i <= y[i]

    # Distance constraint
    model += pl.lpSum(dist[(i, j)] * x[(i, j)] for (i, j) in arcs) <= L

    # MTZ constraints (subtour elimination)
    M = n
    for (i, j) in arcs:
        if i == start:
            continue
        if j == start:
            continue
        model += u[i] - u[j] + 1 <= M * (1 - x[(i, j)])

    # Linking
    model += y[start] == 1

    # Solve
    model.solve(pl.PULP_CBC_CMD(msg=False))

    # Extract solution
    status = pl.LpStatus[model.status]
    print("Solver status:", status)

    chosen_arcs = [(i, j) for (i, j) in arcs if pl.value(x[(i, j)]) > 0.5]
    chosen_nodes = [i for i in key_nodes if pl.value(y[i]) > 0.5]
    best_score = pl.value(model.objective)
    total_length = sum(dist[a] for a in chosen_arcs)

    # Reconstruct tour
    succ = {}
    for (i, j) in chosen_arcs:
        succ[i] = j

    tour = [start]
    cur = start
    while cur in succ:
        cur = succ[cur]
        tour.append(cur)

    # Convert tour to detailed roadmap path
    full_path = []
    for a, b in zip(tour[:-1], tour[1:]):
        seg, _ = shortest_path(a, b, V, E)
        if not full_path:
            full_path.extend(seg)
        else:
            full_path.extend(seg[1:])

    best = {"best_score": best_score, "best_tour": tour, "best_length": total_length}
    return best, full_path



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
    prm = PRM(n_samples=200, k=10, resolution=resolution, seed=0)

    # 1. Build roadmap V = Vertex, E = Edges
    V, E = prm.construct_roadmap(map)

    #OP params
    victims_real = [(9.1, 9.5), (6.7, 5.4), (1.36, 6.34)]
    start_real = (1, 1)
    scores_real  = [500, 200, 150]
    victims = snap_to_nearest_node(victims_real, V)
    start = snap_to_nearest_node(start_real, V)
    # Build score dictionary: associate score to victims
    scores = {v_node: s for v_node, s in zip(victims, scores_real)}
    L = 25.0  # distance/time budget
    # Then call solver
    best_result, full_path = solve_orienteering_brute_force(V, E, victims, scores, start, L)

    #best_result, full_path = solve_orienteering_pulp(V, E, victims, scores, start, L)

    visualization(V, start, victims,best_result,  full_path)
    print(best_result)