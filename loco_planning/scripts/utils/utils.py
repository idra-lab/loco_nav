import numpy as np

class Utils:

    def __init__(self):
        self.crd = {
            "X": 0,
            "Y": 1,
            "Z": 2,
        }

        self.sp_crd = {
            "LX": 0,
            "LY": 1,
            "LZ": 2,
            "AX": 3,
            "AY": 4,
            "AZ": 5,
        }
#no diagonal steps
# def getNeighbors(u, map, th=0.3):
#     neighbors=[]
#     for delta in ((0,1), (0,-1), (1,0), (-1,0)):
#         cand = (u[0]+delta[0] , u[1]+delta[1])
#         if (cand[0]>=0 and cand[0]<len(map) and cand[1]>=0 and cand[1]<len(map[0]) and map[cand[0]][cand[1]] <th): #if is not out of map and is not occupied give me the neighbor
#             neighbors.append(cand)
#     return neighbors

def get8Neighbors(u, map, th=0.3):
    neighbors=[]
    for delta in ((0,1), (0,-1), (1,0), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)):
        cand = (u[0]+delta[0] , u[1]+delta[1])
        if (cand[0]>=0 and cand[0]<len(map) and cand[1]>=0 and cand[1]<len(map[0]) and map[cand[0]][cand[1]] <th): #if is not out of map and is not occupied give me the neighbor
            neighbors.append(cand)
    return neighbors

def get8NeighborsCost(input, map, th=0.3):
    neighbors=[]
    for delta in ((0,1), (0,-1), (1,0), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)):
        cand = (input[0]+delta[0] , input[1]+delta[1])
        cost = np.sqrt(delta[0]**2+delta[1]**2)
        if (cand[0]>=0 and cand[0]<len(map) and cand[1]>=0 and cand[1]<len(map[0]) and map[cand[0]][cand[1]] <th): #if is not out of map and is not occupied give me the neighbor

            neighbors.append((cand, cost))
    return neighbors

  #Helpers
def discretize_segment(self, p0, p1, spacing):
    """Return points along segment from p0->p1 inclusive of p0, exclusive of p1."""
    x0, y0 = p0
    x1, y1 = p1
    L = math.hypot(x1 - x0, y1 - y0)
    if L < 1e-9:
        return []
    n = max(1, int(L // spacing))  # number of intervals
    ts = np.linspace(0.0, 1.0, n + 1)[:-1]  # exclude 1.0 to avoid duplicating next edge's start
    xs = x0 + (x1 - x0) * ts
    ys = y0 + (y1 - y0) * ts
    return list(zip(xs, ys))

def discretize_polygon(self,vertices, spacing):
    """
    vertices: list of (x,y) in order (closed or open).
    Returns a list of sampled (x,y) along the edges.
    """
    pts = []
    m = len(vertices)
    for i in range(m):
        p0 = vertices[i]
        p1 = vertices[(i + 1) % m]
        pts.extend(self.discretize_segment(p0, p1, spacing))
    return pts

def discretize_circle(self,cx, cy, r, spacing, n_min=12):
    circumference = 2 * math.pi * r
    n = max(n_min, int(circumference // spacing))
    angles = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in angles]



