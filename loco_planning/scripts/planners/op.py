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

def solve_orienteering_brute_force(V, E, victims, scores, start, end, L):
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
    key_nodes = [start] + victims + [end]
    sp_dist = {n:  dijkstra(n, V, E)[0]  for n in key_nodes} #[0] is the distance

    # Brute-force small number of victims
    #initialize
    best = {"best_score": -1, "best_tour": None, "best_length": None}
    # Try all victim visit permutations (no return)
    for k in range(0, len(victims) + 1):
        for subset in itertools.combinations(victims, k):
            for perm in itertools.permutations(subset):
                seq = [start] + list(perm) + [end]
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

def visualization(V, start, end, victims, best, full_path):
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
            plt.text(node[0] + 0.2, node[1] + 0.2, f"start", fontsize=10)
        elif node == end:
            plt.scatter(node[0], node[1], color="black", marker="s", s=100, edgecolor="black", zorder=5)
            plt.text(node[0] + 0.2, node[1] + 0.2, f"end", fontsize=10)
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


def solve_orienteering_pulp(V, E, victims, scores, start, end, L):
    """
    Orienteering Problem (Vansteenwegen formulation) with given START and END nodes.
    - Collect scores on victim nodes only (not start or end)
    - Must start at 'start' and end at 'end'
    - Uses shortest-path distances on roadmap (V, E)
    """
    import math
    import pulp as pl

    # ---- Build key nodes ----
    key_nodes = [start] + victims + [end]
    N = len(key_nodes)

    # ---- Precompute shortest-path distances ----
    sp_dist = {n: dijkstra(n, V, E)[0] for n in key_nodes}
    dist = {}
    for i in key_nodes:
        for j in key_nodes:
            if i == j:
                continue
            d = sp_dist[i][j]
            if not math.isinf(d):
                dist[(i, j)] = d

    # ---- MILP model ----
    model = pl.LpProblem("Orienteering_Vansteenwegen", pl.LpMaximize)

    # Decision variables
    x = pl.LpVariable.dicts("x", dist.keys(), 0, 1, cat="Binary")  # edge selection
    #They are ordering variables that encode the visit position of node i in the tour
    ## MTZ order vars: The bounds 2≤ui≤N simply ensure that Start node is not assigned a position (it’s fixed to 1 conceptually), End node can take up to position N, victims are in the 2, N position
    u = pl.LpVariable.dicts("u", victims, lowBound=2, upBound=N, cat="Continuous")

    # ---- Objective ----
    # Collect score when leaving victim node (standard Vansteenwegen)
    # Objective: sum_{i=2..N-1} sum_{j=2..N} S_i * x_ij
    objective_terms = []
    for i in key_nodes[1:-1]:  # i = 2 .. N-1  (skip start and end)
        score_i = scores.get(i, 0.0)
        for j in key_nodes[1:]:  # j = 2 .. N    (skip start only)
            if (i, j) in x:
                objective_terms.append(score_i * x[(i, j)])
    model += pl.lpSum(objective_terms)


    # ---- Constraints ----
    # (1) Start and End constraints
    #start exacly one outgoing none incoming
    # Collect all arcs that go OUT of the start node
    outgoing_arcs = []
    for j in key_nodes[1:]:  # j = 2 .. N
        if (start, j) in x:  # check if arc exists in our dictionary
            outgoing_arcs.append(x[(start, j)])  # store that decision variable
    # Create the linear expression ∑ x[start,j] and add constraint = 1
    outgoing_sum = pl.lpSum(outgoing_arcs)
    model += outgoing_sum == 1, "start_out"
    # Collect all arcs that go INTO the start node
    incoming_arcs = []
    for i in key_nodes[1:]:  # j = 2 .. N
        if (j, start) in x:  # check if arc exists
            incoming_arcs.append(x[(j, start)])  # store that variable
    # Create the linear expression ∑ x[i,start] and add constraint = 0
    incoming_sum = pl.lpSum(incoming_arcs)
    model += incoming_sum == 0, "start_in"

    # End: exactly one incoming, none outgoing
    # Collect all arcs that go OUT of the end node
    outgoing_arcs = []
    for i in key_nodes[:-1]:  # i = 1 .. N-1
        if (end, i) in x:  # check if arc exists in our dictionary
            outgoing_arcs.append(x[(end, i)])  # store that decision variable
    # Create the linear expression ∑ x[start,j] and add constraint = 1
    outgoing_sum = pl.lpSum(outgoing_arcs)
    model += outgoing_sum == 0, "end_out"
    # Collect all arcs that go INTO the end node
    incoming_arcs = []
    for i in key_nodes[:-1]:  # i = 1 .. N-1
        if (i, end) in x:  # check if arc exists
            incoming_arcs.append(x[(i, end)])  # store that variable
    # Create the linear expression ∑ x[i,start] and add constraint = 0
    incoming_sum = pl.lpSum(incoming_arcs)
    model += incoming_sum == 1, "end_in"


    # (2) Flow conservation for victims
    for k in victims:
        incoming_arcs = []
        for i in key_nodes[:-1]:  # i = 1 .. N-1
            if (i, k) in x:  # check if arc exists
                incoming_arcs.append(x[(i, k)])  # store that variable
        inflow = pl.lpSum(incoming_arcs)

        outgoing_arcs = []
        for j in key_nodes[1:]:  # i = 2 .. N
            if (k, j) in x:  # check if arc exists in our dictionary
                outgoing_arcs.append(x[(k, j)])  # store that decision variable
        outflow = pl.lpSum(outgoing_arcs)
        model += inflow == outflow, f"flow_balance_{k}"
        model += inflow <= 1, f"visit_once_{k}"

    # (3) Total travel length constraint
    # Create a list to collect all the linear terms
    distance_terms = []
    # Iterate over every arc (i, j) that exists in the model
    for (i, j) in x:
        # Each arc has a known travel distance and a binary variable x[i,j]
        # Append the weighted term: distance * variable
        distance_terms.append(dist[(i, j)] * x[(i, j)] )
    # Sum all terms using PuLP's linear expression builder
    total_distance  = pl.lpSum(distance_terms)
    # Add the budget constraint: total traveled distance ≤ L
    model += total_distance <= L, "budget"

    # (4) MTZ subtour elimination (no subtours among non-start/end nodes)
    for (i, j) in x:
        if i in victims and j in victims:
            model += u[i] - u[j] + 1 <= (N - 1) * (1 - x[(i, j)]), f"mtz_{i}_{j}"

    # ---- Solve ----
    model.solve(pl.PULP_CBC_CMD(msg=False))
    print("Solver status:", pl.LpStatus[model.status])

    # ---- Extract results ----
    arcs_chosen = [(i, j) for (i, j) in x if pl.value(x[(i, j)]) > 0.5]
    best_score = pl.value(model.objective)
    total_length = sum(dist[a] for a in arcs_chosen)

    # Reconstruct ordered route
    succ = {i: j for (i, j) in arcs_chosen}
    tour = [start]
    cur = start
    seen = {start}
    while cur in succ and succ[cur] not in seen:
        cur = succ[cur]
        tour.append(cur)
        seen.add(cur)

    # ---- Expand roadmap path ----
    full_path = []
    for a, b in zip(tour[:-1], tour[1:]):
        seg, _ = shortest_path(a, b, V, E)
        if not full_path:
            full_path.extend(seg)
        else:
            full_path.extend(seg[1:])

    best = {
        "best_score": best_score,
        "best_tour": tour,
        "best_length": total_length
    }
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
    victims_real = [(9.1, 9.5), (6.7, 5.4), (1.36, 6.34), (3, 6.34), (8,9 )]
    start_real = (4, 9)
    end_real = (0,1)
    scores_real  = [500, 200, 150, 300, 40]
    victims = snap_to_nearest_node(victims_real, V)
    start = snap_to_nearest_node(start_real, V)
    end = snap_to_nearest_node(end_real, V)
    # Build score dictionary: associate score to victims
    scores = {v_node: s for v_node, s in zip(victims, scores_real)}
    L = 25.0  # distance/time budget
    # Then call solver
    #best_result, full_path = solve_orienteering_brute_force(V, E, victims, scores, start, end,  L)

    best_result, full_path = solve_orienteering_pulp(V, E, victims, scores, start, end,   L)

    visualization(V, start, end, victims,best_result,  full_path)
    print(best_result)