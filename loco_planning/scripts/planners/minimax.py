import numpy as np

from planners.prm import PRM
import math
import itertools
import matplotlib.pyplot as plt
import heapq
import time
import matplotlib.animation as animation

'''This is a turn-based model:

MAX (pursuer) and MIN (evader) move alternately, one edge per move.

State = (p_idx, e_idx) (indices of nodes in V).

Terminal:

    p_idx == e_idx → pursuer wins (+1)

    e_idx == gate_idx → evader wins (−1)

    depth limit reached → draw (0)
'''

class PursuitEvasionGame:
    def __init__(self, V, E, gate_idx, max_depth=10, use_alpha_beta=False):
        # convert to numpy array once, useful for heuristic computation
        self.V = np.array(V)
        self.adj = E
        self.gate_idx = gate_idx
        self.max_depth = max_depth
        self.use_alpha_beta = use_alpha_beta
        # transposition table: (p_idx, e_idx, player, depth) -> (value, best_action)
        self.cache = {}
        self.expanded_nodes = 0

    def is_terminal(self, p_idx, e_idx, depth_left):
        #pursuer catches evader (perfect condition)
        # if p_idx == e_idx:
        #     return True
        # pursuer catches evader (catch condition)
        p_pos = self.V[p_idx]
        e_pos = self.V[e_idx]
        # Catch if within distance threshold
        if np.linalg.norm(p_pos - e_pos) < 0.3:
            return True  # pursuer wins

        if e_idx == self.gate_idx:
            return True
        if depth_left == 0:
            return True
        return False

    def utility(self, p_idx, e_idx, depth_left):
        """
           +1: pursuer wins (catch)
           -1: evader wins (reaches gate)
            heuristic: depth cutoff (non-terminal)
           """
        # Terminal: pursuer catches evader (perfect condition)
        # if p_idx == e_idx:
        #     return 1.0

        # Terminal: pursuer catches evader (catch condition)
        p_pos = self.V[p_idx]
        e_pos = self.V[e_idx]
        # Catch condition
        if np.linalg.norm(p_pos - e_pos) < 0.2:
            return 1.0  # pursuer wins

        # Terminal: evader reaches gate
        if e_idx == self.gate_idx:
            return -1.0
        # Depth cutoff: evaluate position
        if depth_left == 0:
            return self.heuristic(p_idx, e_idx)

        # This should not normally be used, but good for safety
        return self.heuristic(p_idx, e_idx)

    # ---------- Minimax wrappers ----------

    def minimax_decision(self, p_idx, e_idx, depth_left, player):
        """
        player = 'P' for pursuer (MAX)
               = 'E' for evader (MIN)
        Returns: best_next_idx (for the player to move), value
        """
        if player == 'P':
            value, action = self.max_value(p_idx, e_idx, depth_left)
        else:
            value, action = self.min_value(p_idx, e_idx, depth_left)

        return action, value

    def max_value(self, p_idx, e_idx, depth_left, alpha = -math.inf, beta=math.inf):
        self.expanded_nodes += 1
        key = (p_idx, e_idx, 'P', depth_left)
        if key in self.cache:
            return self.cache[key]

        if self.is_terminal(p_idx, e_idx, depth_left):
            v = self.utility(p_idx, e_idx, depth_left)
            self.cache[key] = (v, None)
            return v, None

        v = -math.inf
        best_action = None

        for p_next in self.adj[p_idx]:
            v2, _ = self.min_value(p_next, e_idx, depth_left - 1, alpha, beta)
            if v2 > v:
                v = v2
                best_action = p_next

            if self.use_alpha_beta:
                alpha = max(alpha, v)
                if beta <= alpha:
                    break  # prune

        self.cache[key] = (v, best_action)
        return v, best_action

    def min_value(self, p_idx, e_idx, depth_left, alpha = -math.inf, beta=math.inf):
        self.expanded_nodes += 1
        key = (p_idx, e_idx, 'E', depth_left)
        if key in self.cache:
            return self.cache[key]

        if self.is_terminal(p_idx, e_idx, depth_left):
            v = self.utility(p_idx, e_idx, depth_left)
            self.cache[key] = (v, None)
            return v, None

        v = math.inf
        best_action = None

        for e_next in self.adj[e_idx]:
            v2, _ = self.max_value(p_idx, e_next, depth_left - 1, alpha, beta)
            if v2 < v:
                v = v2
                best_action = e_next

            if self.use_alpha_beta:
                beta = min(beta, v)
                if beta <= alpha:
                    break  # prune

        self.cache[key] = (v, best_action)
        return v, best_action

    def heuristic(self, p_idx, e_idx):
        V = self.V
        d_e_gate = np.linalg.norm(V[e_idx] - V[self.gate_idx])
        d_p_e = np.linalg.norm(V[p_idx] - V[e_idx])

        # MAX = pursuer, MIN = evader
        # Pursuer wants: d_e_gate small, d_p_e small
        # Evader wants: d_e_gate small, d_p_e large
        # Negative makes MAX want to minimize this score, MIN will naturally minimize
        return -d_e_gate - 0.2 * d_p_e

# helpers
def nearest_node(V, point):
    """Return index of roadmap node closest to 'point' (x, y)."""
    V_arr = np.asarray(V)
    p = np.array(point)
    d2 = np.sum((V_arr - p) ** 2, axis=1)
    return int(np.argmin(d2))

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

def solve_minmax( V, E, gate, p_start_world, e_start_world, max_depth=10, use_alpha_beta=False):
    """
    V, E: roadmap (from PRM)
    gate: (x, y) of gate in world coords
    p_start_world, e_start_world: (x, y) starting positions in world coords
    max_depth: maximum number of *ply* (half-moves)
    Returns:
        p_path_idx: list of pursuer node indices along play
        e_path_idx: list of evader node indices along play
        winner: 'P', 'E', or 'Draw'
        value: minimax utility (+1, -1, 0)
    """
    gate_idx = nearest_node(V, gate)
    p_idx = nearest_node(V, p_start_world)
    e_idx = nearest_node(V, e_start_world)

    game = PursuitEvasionGame(V, E, gate_idx, max_depth=max_depth, use_alpha_beta=use_alpha_beta)

    p_path = [p_idx]
    e_path = [e_idx]

    player = 'P'  # pursuer starts (can change if you want)
    depth_left = max_depth
    winner = 'Draw'
    value = 0.0

    while not game.is_terminal(p_idx, e_idx, depth_left):
        action, value = game.minimax_decision(p_idx, e_idx, depth_left, player)
        if action is None:
            # no move or terminal reached by evaluation
            break

        if player == 'P':
            p_idx = action
            p_path.append(p_idx)
            player = 'E'
        else:
            e_idx = action
            e_path.append(e_idx)
            player = 'P'

        depth_left -= 1

    # Determine winner from final state
    winner, final_util, reason = determine_winner(np.array(V), gate_idx, p_idx, e_idx, depth_left)

    return p_path, e_path, winner, final_util, gate_idx, game.expanded_nodes, reason

def determine_winner(V, gate_idx, p_idx, e_idx, depth_left, capture_radius=0.2):
    p_pos = V[p_idx]
    e_pos = V[e_idx]

    # 1. Pursuer catches evader?
    if np.linalg.norm(p_pos - e_pos) < capture_radius:
        return 'P', 1.0, "Pursuer caught the evader"

    # 2. Evader reaches gate?
    if e_idx == gate_idx:
        return 'E', -1.0, "Evader reached the gate"

    # 3. Depth limit or stalemate → draw
    if depth_left == 0:
        return 'Draw', 0.0, "Depth limit reached (no terminal event)"

    # Fallback (shouldn't happen)
    return 'Draw', 0.0, "Undefined condition"



'''
This uses:

    - map as binary grid (0 free, 1 obstacle) you already have
    - Roadmap nodes and edges
    - Pursuer path (blue) and evader path (red)
    - Gate node (red)
'''
def visualize(map, resolution, V, E, p_path_idx, e_path_idx, gate_idx,
              map_width=10., map_height=10., winner=None):

    # Plot roadmap edges (light)
    # for i, nbrs in E.items():
    #     for j in nbrs:
    #         x = [V[i][0], V[j][0]]
    #         y = [V[i][1], V[j][1]]
    #         plt.plot(x, y, linewidth=0.3, alpha=0.3)
    #
    # # Plot roadmap vertices
    # V_arr = np.asarray(V)
    # plt.scatter(V_arr[:, 0], V_arr[:, 1], s=5, c='k', alpha=0.4)

    # Plot gate node
    gx, gy = V[gate_idx]
    plt.scatter([gx], [gy],  s=200, color="red", marker='*')

    # Extract paths
    p_path = np.asarray([V[i] for i in p_path_idx])
    e_path = np.asarray([V[i] for i in e_path_idx])

    # Plot paths pursuer
    plt.plot(p_path[:, 0], p_path[:, 1], color="blue", linewidth=3)
    plt.scatter(p_path[0, 0], p_path[0, 1], s=80, color="blue", marker='o')  # pursuer start

    # Plot paths evader
    plt.plot(e_path[:, 0], e_path[:, 1], color="red", linewidth=3)
    plt.scatter(e_path[0, 0], e_path[0, 1], s=80, color="red", marker='s')  # evader start

    # --- Final event marker ---
    p_final = V[p_path_idx[-1]]
    e_final = V[e_path_idx[-1]]
    if winner == 'P':  # pursuer caught evader
        # mark capture location
        plt.scatter([e_final[0]], [e_final[1]], s=250, color="green", edgecolor='black', marker='X', label="Capture")
    elif winner == 'E':  # evader reached gate
        gx, gy = V[gate_idx]
        plt.scatter([gx], [gy], s=250, color="yellow", edgecolor='black', marker='*', label="Escape")
    else:  # draw
        plt.scatter([e_final[0]], [e_final[1]], s=200, color="gray", edgecolor='black', marker='o', label="Draw end")

    plt.xlim(0, map_width)
    plt.ylim(0, map_height)
    plt.gca().set_aspect('equal')
    plt.title("Pursuit–Evasion Minimax Paths")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

def animate_pursuit(map, resolution, V, p_path_idx, e_path_idx, gate_idx,
                    map_width=10., map_height=10., interval_ms=1000):
    """
    Animate pursuer and evader on the PRM graph.
    interval_ms: delay between frames (1 second = 1000 ms)
    """
    fig, ax = plt.subplots(figsize=(6, 6))

    # --- Draw static background ---
    extent = [0, map_width, 0, map_height]
    ax.imshow(map, origin='lower', cmap='gray_r', extent=extent, alpha=0.7)

    # Draw roadmap edges faintly
    for i, nbrs in E.items():
        for j in nbrs:
            x = [V[i][0], V[j][0]]
            y = [V[i][1], V[j][1]]
            ax.plot(x, y, linewidth=0.3, alpha=0.3, color='gray')

    # Draw all nodes
    V_arr = np.asarray(V)
    ax.scatter(V_arr[:, 0], V_arr[:, 1], s=5, c='k', alpha=0.4)

    # Gate
    gx, gy = V[gate_idx]
    ax.scatter([gx], [gy], s=100, marker='*', color='green', label="Gate")

    # --- Animated elements ---
    p_marker, = ax.plot([], [], 'bo', markersize=12, label="Pursuer")
    e_marker, = ax.plot([], [], 'ro', markersize=12, label="Evader")

    ax.set_xlim(0, map_width)
    ax.set_ylim(0, map_height)
    ax.set_aspect('equal')
    ax.set_title("Pursuit–Evasion Animation")
    ax.legend()

    # --- Animation update function ---
    def update(frame):
        # frame = step index
        if frame < len(p_path_idx):
            px, py = V[p_path_idx[frame]]
            p_marker.set_data(px, py)

        if frame < len(e_path_idx):
            ex, ey = V[e_path_idx[frame]]
            e_marker.set_data(ex, ey)

        return p_marker, e_marker

    # Number of frames = length of longest path
    n_frames = max(len(p_path_idx), len(e_path_idx))

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=n_frames,
        interval=interval_ms,
        blit=True,
        repeat=False
    )

    plt.show()


if __name__ == '__main__':
    map_width = 10.
    map_height = 10.
    resolution = 0.1
    map = np.zeros((int(map_width / resolution), int(map_height / resolution)))

    # Obstacles
    set_obstacle(center=(2, 8), size=2, resolution=resolution, map=map)
    set_obstacle(center=(4, 4.5), size=1.5, resolution=resolution, map=map)
    set_obstacle(center=(8, 4), size=1, resolution=resolution, map=map)


    gate = (5, 0)

    # Build roadmap
    prm = PRM(n_samples=100, k=10, resolution=resolution, seed=0)
    V, E = prm.construct_roadmap_index(map)

    # Example start positions in world coordinates
    # pursuer closer to gate than evader
    p_start_world = (4, 2)  # pursuer
    e_start_world = (1, 1)  # evader

    # evader closer to gate than pursuer
    # p_start_world = (1, 1) # pursuer
    # e_start_world =  (6, 2)  # evader

    # simulate a game where both robots play optimally (minimax each turn).
    p_path_idx, e_path_idx, winner, final_util, gate_idx, expanded_nodes, reason = solve_minmax( V, E, gate,
        p_start_world=p_start_world,
        e_start_world=e_start_world,
        max_depth=100,
        use_alpha_beta= False
    )
    print("Nodes expanded:", expanded_nodes)
    print(f"Winner: {winner} | utility: {final_util:.2f} | reason: {reason}")
    visualize(map, resolution, V, E, p_path_idx, e_path_idx, gate_idx,
              map_width=map_width, map_height=map_height, winner=winner)

    # Play 1-second-per-step animation
    animate_pursuit(
        map, resolution, V, p_path_idx, e_path_idx, gate_idx,
        map_width=map_width, map_height=map_height,
        interval_ms=1000   # 1 second per step
    )
