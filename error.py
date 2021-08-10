import shapely.geometry as geom
import matplotlib.pyplot as plt
import numpy as np

def calculate(ground_truth_x, ground_truth_y, estimated_x, estimated_y):
    """Calculates the average error margin for a test"""

    # Apply steps points to every Ground Truth line
    points = np.linspace(start=ground_truth_x, stop=ground_truth_y, num=100)
    total_points = []
    xs, ys= [], []
    # Save all points into an array of (x,y)
    for i in range(len(points) - 1):
        for j in range(len(ground_truth_x)):
            total_points.append((points[0][j],points[i+1][j]))
            total_points.append((points[i+1][j],points[0][j]))
            xs.append(points[0][j])
            ys.append(points[i+1][j])
    # plt.scatter(xs,ys)
    # plt.scatter(ys,xs)
    # Create a Shapely LineString from totality of Ground truth points
    ground_truth = geom.LineString(total_points)

    # For every estimated point calculate the distance to the closest neighbor in the Ground Truth LineString
    for i in range(len(estimated_x)):
        # Create a Shapely Point of the estimated sample
        estimated_point = geom.Point(estimated_x[i],estimated_y[i])

        # Calculate the distance from the estimated sample point to ground truth
        closest_distance = estimated_point.distance(ground_truth)
        closest_point = ground_truth.interpolate(ground_truth.project(estimated_point))
        # plt.plot([estimated_point.x, closest_point.x], [estimated_point.y, closest_point.y])