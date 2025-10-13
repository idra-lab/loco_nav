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

def getNeighbors(u, map, th=0.3):
    neighbors=[]
    for delta in ((0,1), (0,-1), (1,0), (-1,0)):
        cand = (u[0]+delta[0] , u[1]+delta[1])
        if (cand[0]>=0 and cand[0]<len(map) and cand[1]>=0 and cand[1]<len(map[0]) and map[cand[0]][cand[1]] <th): #if is not out of map and is not occupied give me the neighbor
            neighbors.append(cand)
    return neighbors

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



            