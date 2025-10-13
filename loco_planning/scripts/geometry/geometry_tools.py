import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.optimize import linprog, minimize
import numpy as np

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.optimize import linprog, minimize


def vertex_to_halfspace_description(V, draw=False):
    """
    Given a polygon (list of vertices V), return half-space representation (A, b)
    such that A x <= b describes the convex hull of V.
    """
    V = np.array(V)
    centroid = np.mean(V, axis=0)
    hull = ConvexHull(V)
    vertices = hull.vertices  # ordered CCW

    A, b = [], []

    if draw:
        plt.figure()
        plot_polygon(V[vertices], color = 'r')

    for i in range(len(vertices)):
        i_next = (i + 1) % len(vertices)
        x0, y0 = V[vertices[i]]
        x1, y1 = V[vertices[i_next]]

        if draw:
            plt.plot([x0, x1], [y0, y1], '--rs')

        # Normal vector
        normal = np.array([y1 - y0, -(x1 - x0)])
        bc = (y1 - y0) * x0 - (x1 - x0) * y0

        # Orientation check
        S = np.sign(-(normal @ centroid) + bc)
        A.append(S * normal)
        b.append(S * bc)

    return np.array(A), np.array(b)


def point_inside_polygon(point, V):
    """
    Check if point is inside convex polygon defined by V.
    """
    A, b = vertex_to_halfspace_description(V, draw=True)
    residual = A @ point - b
    return np.all(residual <= 1e-9), residual


def polygon_intersect(V1, V2, draw=False):
    """
    Check if convex polygons intersect by solving feasibility problem.
    """
    A1, b1 = vertex_to_halfspace_description(V1, False)
    A2, b2 = vertex_to_halfspace_description(V2, False)

    A = np.vstack((A1, A2))
    b = np.hstack((b1, b2))

    res = linprog(c=np.zeros(2), A_ub=A, b_ub=b)

    if draw:
        hull1 = ConvexHull(V1)
        hull2 = ConvexHull(V2)
        plt.figure()
        plot_polygon(V1[hull1.vertices], color='r')
        plot_polygon(V2[hull2.vertices], color='b')
        plt.axis("equal")
        plt.grid(True)

    return res.success

def plot_polygon(V, color='r'):
    """
    Plot a convex polygon defined by vertices V.
    Assumes CCW ordering and closes the polygon.
    """
    #Draw polygon vertices
    plt.plot(V[:, 0], V[:, 1], 'o')

    # Draw polygon edges
    plt.plot(V[:, 0], V[:, 1], '--'+color)
    # Close polygon
    plt.plot([V[-1, 0], V[0, 0]],
             [V[-1, 1], V[0, 1]], '--'+color)
    plt.grid(True)
    plt.axis("equal")

def distance_point_to_polygon(point, V, draw=False):
    """
    Compute closest point on polygon V to given point.
    """
    point = np.array(point).reshape(2)

    A, b = vertex_to_halfspace_description(V, False)

    def objective(x):
        return 0.5 * np.dot(x - point, x - point)

    cons = [{"type": "ineq", "fun": lambda x, Arow=A[i], brow=b[i]: brow - Arow @ x} for i in range(len(A))]
    res = minimize(objective, x0=point, constraints=cons)

    if not res.success:
        raise RuntimeError("Optimization failed")

    closest_point = res.x
    distance = np.linalg.norm(point - closest_point)

    if draw:
        hull = ConvexHull(V)
        plt.figure()
        plot_polygon(V[hull.vertices], color='r')
        plt.plot(point[0], point[1], 'bx')
        plt.plot(closest_point[0], closest_point[1], 'ob')
        plt.plot([point[0], closest_point[0]], [point[1], closest_point[1]], '-b')
        plt.axis("equal")
        plt.grid(True)

    return closest_point, distance


def distance_between_polygons(V1, V2, draw=False):
    """
    Compute closest distance between two convex polygons.
    """
    A1, b1 = vertex_to_halfspace_description(V1, False)
    A2, b2 = vertex_to_halfspace_description(V2, False)

    def objective(x):
        p1 = x[:2]
        p2 = x[2:]
        return 0.5 * np.dot(p1 - p2, p1 - p2)

    cons = []
    for i in range(len(A1)):
        cons.append({"type": "ineq", "fun": lambda x, Arow=A1[i], brow=b1[i]: brow - Arow @ x[:2]})
    for i in range(len(A2)):
        cons.append({"type": "ineq", "fun": lambda x, Arow=A2[i], brow=b2[i]: brow - Arow @ x[2:]})

    res = minimize(objective, x0=np.zeros(4), constraints=cons)

    if not res.success:
        raise RuntimeError("Optimization failed")

    closest_point1 = res.x[:2]
    closest_point2 = res.x[2:]
    distance = np.linalg.norm(closest_point1 - closest_point2)

    if draw:
        hull1 = ConvexHull(V1)
        hull2 = ConvexHull(V2)
        plt.figure()
        plot_polygon(V1[hull1.vertices], color = 'r')
        plot_polygon(V2[hull2.vertices], color = 'b')
        plt.plot(closest_point1[0], closest_point1[1], 'ob')
        plt.plot(closest_point2[0], closest_point2[1], 'ob')
        plt.plot([closest_point1[0], closest_point2[0]],
                 [closest_point1[1], closest_point2[1]], '-b')
        plt.axis("equal")
        plt.grid(True)

    return closest_point1, closest_point2, distance

if __name__ == "__main__":

    # Example polygon
    P = np.array([
        [0, 0],
        [1, 1],
        [1.5, 0.5],
        [1.5, -0.5],
        [1.25, 0.3],
        [1, 0],
        [1.25, -0.3],
        [1, -1]
    ])

    plt.figure()
    plt.plot(P[:, 0], P[:, 1], '*')
    plt.grid(True)
    hull = ConvexHull(P)
    plot_polygon(P[hull.vertices])
    plt.title("Convex Hull Example")

    # # Example 1: vertex to halfspace
    A, b = vertex_to_halfspace_description(P, False)

    # Example 2: point inside polygon
    result, residual = point_inside_polygon(np.array([1, 0]), P)
    print("Inside?", result, "Residual:", residual)

    # # Example 3: intersection test
    V1 = np.random.rand(20, 2) * 5 + [3.0, 3.0]
    V2 = np.random.rand(20, 2) * 6 + [-1, -1]
    print("Intersect?", polygon_intersect(V1, V2, draw=True))
    #
    # # Example 4: distance from point to polygon
    V1 = np.random.rand(20, 2) * 5 + [3.0, 3.0]
    point = np.random.rand(2) * 5 + [-1, -1]
    closest_point, distance = distance_point_to_polygon(point, V1, draw=True)
    print("Closest point:", closest_point, "Distance:", distance)
    #
    # # Example 5: distance between polygons
    V1 = np.random.rand(20, 2) * 5 + [3.0, 3.0]
    V2 = np.random.rand(20, 2) * 4 + [-1, -1]
    closest_point1, closest_point2, distance = distance_between_polygons(V1, V2, draw=True)
    print("Closest pair:", closest_point1, closest_point2, "Distance:", distance)
    plt.show()
