from ahrs.filters import AngularRate, AQUA, Complementary, Davenport, EKF, FAMC, FLAE, Fourati, FQA, Madgwick, Mahony, OLEQ, QUEST, ROLEQ, SAAM, Tilt
from squaternion import Quaternion
import integrations
import numpy as np
import math
import error
import turn
import plotting
import matplotlib.pyplot as plt

# Read data from files
data = np.loadtxt("data.txt")
acc_data, rest = np.hsplit(data, [3])
acc_data *= 9.8
gyr_data, rest = np.hsplit(rest, [3])
gyr_data *= 0.017453
mag_data, dist_data = np.hsplit(rest, [3])
mag_data /= 1000

# algorithms = [AngularRate, AQUA, Complementary, Davenport, EKF, FAMC, FLAE, Fourati, FQA, Madgwick, Mahony, OLEQ, QUEST, ROLEQ, SAAM, Tilt]

algorithms = [Mahony]
sample_error = []
for algorithm in algorithms:
    for frequency in np.arange(1, 2.0, 1):
        # Saving ahrs estimations to array
        estimation = []

        # Start ahrs estimation
        if (
            algorithm == FAMC
            or algorithm == FLAE
            or algorithm == FQA
            or algorithm == OLEQ
            or algorithm == QUEST
            or algorithm == SAAM
            or algorithm == Tilt
        ):
            ahrs = algorithm(acc=acc_data, mag=mag_data)  # Returns Quaternions
        else:
            ahrs = algorithm(acc=acc_data, gyr=gyr_data, mag=mag_data, frequency=frequency)  # Returns Quaternions

        for t in range(1, len(data)):
            q = Quaternion(ahrs.Q[t][0], ahrs.Q[t][1], ahrs.Q[t][2], ahrs.Q[t][3])
            estimation.append(q.to_euler(degrees=True))

        # Convert estimation to numpy array
        estimation = np.array(estimation)

        positions = []

        for index, distance in enumerate(dist_data, start=-1):
            # Calculate X, Y, Z positions - Distance * heading angle. Update the coordinate with each function
            coordinateX, coordinateY, coordinateZ = integrations.getCoordinate()
            positionX = distance * math.cos(math.radians(estimation[index][2])) + coordinateX
            positionY = distance * math.sin(math.radians(estimation[index][2])) + coordinateY
            positionZ = distance * math.sin(math.radians(estimation[index][0])) + coordinateZ
            positions.append([positionX, positionY, positionZ])
            integrations.updateCoordinates(positionX, positionY, positionZ)

        integrations.updateCoordinates(0, 0, 0)

        x, y, z = [], [], []

        for position in positions:
            x.append(-position[0])
            y.append(position[1])
            z.append(position[2] / 30)

        # Calculate turning point
        trajectory_points = turn.trajectory(x,y)
        x_trajectory, y_trajectory = [],[]

        for point in trajectory_points:
            x_trajectory.append(point[0])
            y_trajectory.append(point[1])

        # GROUND TRUTH - Square, triangle, spiral, etc.
        ground_truth = [[0, 0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16, 0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0], [ 0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16, 0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0, 0], [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]] # Squares
        # ground_truth = [[0, 0, 28, 28, 0], [ 0, 28, 28, 0, 0], [ 0, 0, 0, 0, 0]] # Square
        # ground_truth = [[0, 0, 28, 0], [ 0, 28, 0, 0], [ 0, 0, 0, 0]] # Triangle
        # ground_truth = [[0, 28], [0, 0], [0, 0]] # Line

        # Error measurement for every point
        # experiment_error = error.calculate(ground_truth[0],ground_truth[1], x, y)

        # Error measurement for every corner
        # turn_error = error.calculate_turn(ground_truth[0],ground_truth[1], x_trajectory, y_trajectory)

        # sample_error.append({'Frequency' : frequency, 'Experiment Error' : experiment_error, 'Turn Error' : turn_error})

        # PLOTTING
        plotting.plot2D(ground_truth[0],ground_truth[1], x, y, x_trajectory, y_trajectory)
        # plotting.video2D(x, y)
        # plotting.plot3D(ground_truth[0],ground_truth[1], ground_truth[2], x, y, z, x_trajectory, y_trajectory)
        # plotting.video3D(x, y, z)

# plt.plot( [frequency['Frequency'] for frequency in sample_error],  [error['Experiment Error'] for error in sample_error])
# plt.plot( [frequency['Frequency'] for frequency in sample_error],  [error['Turn Error'] for error in sample_error])
# plt.xlabel('Sample frequency (Hz)')
# plt.ylabel('Average error (m)')
# plt.show()