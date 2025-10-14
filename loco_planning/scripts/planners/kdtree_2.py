import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from math import dist
from scipy.spatial import KDTree

class Node:
    def __init__(self, point, axis, left=None, right=None):
        self.point = point
        # splitting axis
        self.axis = axis
        self.left = left
        self.right = right

def build_kdtree(points, depth=0):
    if not points:
        return None
    #k is the number of dimension per point
    k = len(points[0])
    #If we always split on the same axis (say x), the tree could become deep and narrow, and not reduce search time much
    # depth will increase by one each time so you will have alternation, this makes the multi-dimensional tree balanced
    axis = depth % k

    # sort point by axis coordinate (X or Y)
    points = sorted(points, key=lambda p: p[axis])
    median = len(points) // 2 #median is the integer division by 2
    return Node(
        points[median], axis,
        build_kdtree(points[:median], depth + 1),
        build_kdtree(points[median + 1:], depth + 1)
    )



def nearest_neighbor(root, target, depth=0, best=None):
    if root is None:
        return best
    if best is None or dist(target, root.point) < dist(target, best):
        best = root.point
    axis = root.axis
    if target[axis] < root.point[axis]:
        next_branch, opposite = root.left, root.right
    else:
        next_branch, opposite = root.right, root.left
    best = nearest_neighbor(next_branch, target, depth + 1, best)
    if abs(target[axis] - root.point[axis]) < dist(target, best):
        best = nearest_neighbor(opposite, target, depth + 1, best)
    return best


def plot_kdtree(node, xmin, xmax, ymin, ymax, depth=0):
    """Recursively plot KD-tree splitting lines"""
    if node is None:
        return
    x, y = node.point
    axis = node.axis

    if axis == 0:
        # vertical split
        plt.plot([x, x], [ymin, ymax], 'r--', linewidth=1)
        plot_kdtree(node.left, xmin, x, ymin, ymax, depth + 1)
        plot_kdtree(node.right, x, xmax, ymin, ymax, depth + 1)
    else:
        # horizontal split
        plt.plot([xmin, xmax], [y, y], 'b--', linewidth=1)
        plot_kdtree(node.left, xmin, xmax, ymin, y, depth + 1)
        plot_kdtree(node.right, xmin, xmax, y, ymax, depth + 1)

    plt.plot(x, y, 'go')  # draw node

if __name__ == "__main__":
    # Example data
    points = [(2, 3), (5, 4), (9, 6), (4, 7), (8, 1), (7, 2)]
    query = (9, 2)


    # # Build tree and find nearest neighbor: custom implem
    tree = build_kdtree(points)
    nearest = nearest_neighbor(tree, query)
    radius = dist(query, nearest)
    print(f"CUSTOM: Nearest to {query} is {nearest}")

    # Plot
    plt.figure(figsize=(6, 6))
    # Draw KD-tree partition lines
    plot_kdtree(tree, xmin=0, xmax=10, ymin=0, ymax=10)
    for p in points:
        plt.scatter(p[0], p[1], color='g',label='Dataset Points' if p == points[-1] else None)
    plt.scatter(query[0],query[1], color='red', s=80, label='Query')
    plt.scatter(nearest[0],nearest[1], color='blue', s=120, label='Nearest Neighbor')


    # Draw search circle
    circle = plt.Circle(query, radius, color='orange', fill=False, lw=1.5)
    plt.gca().add_patch(circle)
    plt.legend()
    plt.title("2D k-dimensional binary tree Visualization")
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

    # compare with SCIPY library implementation
    tree_sci = KDTree(points)
    dist, idx = tree_sci.query(query)
    print(f"SCIPY: Nearest to {query} is {nearest}")


