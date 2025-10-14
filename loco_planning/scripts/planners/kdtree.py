import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from math import dist
from scipy.spatial import KDTree as sciKDTree
import numpy as np

axis_label=['X','Y']
class Node:
    def __init__(self, axis=None, value=None, left=None, right=None, point=None, label=None):
        self.axis = axis    # splitting axis (int)
        self.value = value  # splitting coordinate (float)
        self.left = left    # left subtree (Node or None)
        self.right = right  # right subtree (Node or None)
        self.point = point  # data point (tuple) only if node is a leaf
        self.label = label

class KDTree():
    def __init__(self, points = None, bounds = None):
        if points is not None:
            if bounds is None:
                # Compute bounding box automatically from dataset
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                xmin, xmax = min(xs), max(xs)
                ymin, ymax = min(ys), max(ys)
                bounds = (xmin, xmax, ymin, ymax)
            self.tree = self.build_kdtree(points, bounds=bounds, debug=False)

    # ---------- KD-Tree Builder with Cells ----------
    def build_kdtree(self, points, depth=0, bounds=(0,10,0,10), debug = False, counter=[1]):
        """
        Builds a KD-tree recursively. A KD-tree (binary tree splitting space by coordinates), A query point q.
        We want to find the closest point to q among all nodes. The naive way would be to check all points — but the KD-tree helps us skip large parts of the search space using geometry.
        - points: list of (x, y) tuples
        - depth: recursion level
        - bounds: (xmin, xmax, ymin, ymax) used for visualization
        """
        if not points:
            return None

        xmin, xmax, ymin, ymax = bounds
        # k is the number of dimension per point
        k = len(points[0])
        # If we always split on the same axis (say x), the tree could become deep and narrow, and not reduce search time much
        # depth will increase by one each time so you will have alternation, this makes the multi-dimensional tree balanced
        axis = depth % k



        # Stop when only one point left → leaf node
        # ---- LEAF ----
        if len(points) == 1:
            x, y = points[0]
            # plt.gca().add_patch(plt.Rectangle((xmin, ymin),
            #                                   xmax - xmin, ymax - ymin,
            #                                   fill=False, edgecolor='gray', lw=1))
            plt.scatter(x, y, color='blue', zorder=5)

            if debug:
                plt.draw()
                plt.pause(0.01)
                plt.show(block=False)
                input(f"leaf ({x}, {y}) reached. Press Enter to continue...")
            return Node(point=points[0] )

        # Sort by current axis and find median value
        # ---- INTERNAL NODE ----
        points = sorted(points, key=lambda p: p[axis])
        median_index = len(points) // 2

        # Split BETWEEN neighboring points (not through one)
        left_points = points[:median_index]
        right_points = points[median_index:]
        # last of the left points
        split_left = left_points[-1][axis]
        # first of the right points
        split_right = right_points[0][axis]
        # take a value in the middle
        value = 0.5 * (split_left + split_right)

        label = f"s{counter[0]}"
        counter[0] += 1

        # Draw split line
        if axis == 0:  # vertical
            plt.plot([value, value], [ymin, ymax], 'r--', lw=1)
            plt.text(value-0.3, ymax-0.5, label, color='red', fontsize=9,
                     ha='center', va='bottom', backgroundcolor='white')
            left_bounds = (xmin, value, ymin, ymax)
            right_bounds = (value, xmax, ymin, ymax)
            if debug:
                plt.draw()
                plt.pause(0.01)
                plt.show(block=False)
                input(f"{label}: Splitting axis {axis_label[axis]} at {value} (vertically). Press Enter to continue...")
        else:  # horizontal parallel to X axis
            plt.plot([xmin, xmax], [value, value], 'b--', lw=1)
            plt.text(xmax-0.5, value-0.3, label, color='blue', fontsize=9, ha='left', va='center', backgroundcolor='white')
            left_bounds = (xmin, xmax, ymin, value)
            right_bounds = (xmin, xmax, value, ymax)

            if debug:
                plt.draw()
                plt.pause(0.01)
                plt.show(block=False)
                input(f"{label}: Splitting axis {axis_label[axis]} at {value} (horizontally). Press Enter to continue...")

        # Recursive build for subtrees
        points_in_left_child = points[:median_index]
        points_in_right_child = points[median_index:]
        if debug:
            print(f"   Left Child points: {points_in_left_child}")
            print(f"   Right Child points: {points_in_right_child}")
        left_child = self.build_kdtree(points_in_left_child, depth + 1, left_bounds, debug=debug)
        right_child = self.build_kdtree(points_in_right_child, depth + 1, right_bounds,debug=debug)

        return Node(axis=axis, value=value, left=left_child, right=right_child, label = label)

    def nearest_neighbor2(self, query, tree,  depth=0, best_point=None):
        """
        KD-tree nearest neighbor search simpler implementation

        query : query point (tuple)
        tree : current KD-tree node
        depth :
        best_point : current best (nearest) point
        best_dist : current best distance
        """
        if tree is None:
            return best_point

        # ----- Case 1: Leaf node -----
        if tree.point is not None:
            # Evaluate distance to this point
            if best_point is None or dist(query, tree.point) < dist(query, best_point):
                best_point = tree.point
                best_dist = dist(query, tree.point)
            return best_point, best_dist

        # ----- Case 2: Internal node -----
        axis = tree.axis
        value = tree.value

        # Choose near/far branch based on the splitting value
        if query[axis] < value:
            near_branch, far_branch = tree.left, tree.right
        else:
            near_branch, far_branch = tree.right, tree.left

        # Recurse into nearer branch first
        best_point, best_dist = self.nearest_neighbor2(query, near_branch,  depth + 1, best_point)

        # Check if the hypersphere crosses the split plane
        if abs(query[axis] - value) < dist(query, best_point):
            best_point, best_dist = self.nearest_neighbor2(query, far_branch,  depth + 1, best_point)

        return best_point, best_dist


    def nearest_neighbor(self, query, node,  best_point=None, best_dist=float("inf")):
        """
        KD-tree nearest neighbor search based on Class algorithm. Searches only necessary branches — prunes others. search_first branch first
        then overlap check (q(axis) ± w vs n.value).

        query : query point (tuple)
        node : current KD-tree node
        best_point : current best (nearest) point
        best_dist : current best distance
        """

        # ----- Base / leaf case -----
        if node is None:
            return best_point, best_dist
        # Leaf node: Compute direct distance to the query point and possibly update best_point.
        if node.left is None and node.right is None and node.point is not None:
            d = dist(query, node.point)
            if d < best_dist:
                return node.point, d
            else:
                return best_point, best_dist

        # ----- Internal node -----
        axis = node.axis
        split_val = node.value

        # Decide which side to search first
        if query[axis] <= split_val:
            # search_first := left
            # Fisrt Search left side (where query lies)
            best_point, best_dist = self.nearest_neighbor(query, node.left, best_point, best_dist)
            # Then Check Overlap: check If the search circle (radius = current best distance) crosses the split line on the right,
            if query[axis] + best_dist > split_val:  # circle overlaps RIGHT half-space we need to continue search on the right
                best_point, best_dist = self.nearest_neighbor(query, node.right, best_point, best_dist)

        else:
            # search_first := right
            # first Search right side (where query lies)
            best_point, best_dist = self.nearest_neighbor(query, node.right, best_point, best_dist)
            # Then Check Overlap: check If the search circle (radius = current best distance) crosses the split line on the left,
            if query[axis] - best_dist <=                                                                           split_val:  # circle overlaps LEFT half-space we need to continue search on the left
                best_point, best_dist = self.nearest_neighbor(query, node.left, best_point, best_dist)

        return best_point, best_dist



    def print_tree(self, node, depth=0):
        indent = "  " * depth
        if node is None:
            return
        if node.point is not None:
            print(f"{indent}Leaf: point={node.point}")
        else:
            print(f"{indent}Node: axis={node.axis}, value={node.value:.2f}, label={node.label}")
            self.print_tree(node.left, depth + 1)
            self.print_tree(node.right, depth + 1)


if __name__ == "__main__":
    # Example data
    points = [(2, 3), (5, 4), (9, 6), (4, 7), (8, 1), (7, 2)]
    query = (9, 2)

    plt.figure(figsize=(6, 6))
    for p in points:
        plt.scatter(p[0], p[1], color='g', label='Dataset Points' if p == points[-1] else None)
    plt.scatter(query[0],query[1], color='red', s=80, label='Query')
    plt.xticks(np.arange(0, 11, 1))
    plt.yticks(np.arange(0, 11, 1))
    plt.xlim([0,10])
    plt.ylim([0, 10])
    plt.ylabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    # # Build tree and find nearest neighbor: custom implem
    kdtree = KDTree()
    tree = kdtree.build_kdtree(points, bounds=(0, 10, 0, 10), debug=False)
    nearest, radius = kdtree.nearest_neighbor(query,tree)
    print(f"CUSTOM: Nearest to {query} is {nearest}")

    # Plot
    plt.scatter(nearest[0],nearest[1], color='blue', s=120, label='Nearest Neighbor')
    # Draw search circle
    circle = plt.Circle(query, radius, color='orange', fill=False, lw=1.5)
    plt.gca().add_patch(circle)
    plt.legend()
    plt.title("2D k-dimensional binary tree Visualization")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

    # compare with library implementation
    # compare with SCIPY library implementation
    tree_sci = sciKDTree(points)
    dist, idx = tree_sci.query(query)
    print(f"SCIPY: Nearest to {query} is {points[idx]}")

    #print logical representation
    kdtree.print_tree(tree)
