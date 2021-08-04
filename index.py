from ahrs.filters import AngularRate, AQUA, Complementary, Davenport, EKF, FAMC, FLAE, Fourati, FQA, Madgwick, Mahony, OLEQ, QUEST, ROLEQ, SAAM, Tilt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from squaternion import Quaternion
import matplotlib.pyplot as plt
import integrations
import numpy as np
import math
import error

# Read data from files
data = np.loadtxt("data.txt")
acc_data, rest = np.hsplit(data, [3])
acc_data *= 9.8
gyr_data, rest = np.hsplit(rest, [3])
gyr_data *= 0.017453
mag_data, dist_data = np.hsplit(rest, [3])
mag_data /= 1000

# algorithms = [AngularRate, AQUA, Complementary, Davenport, EKF, FAMC, FLAE, Fourati, FQA, Madgwick, Mahony, OLEQ, QUEST, ROLEQ, SAAM, Tilt]

algorithms = [Madgwick]

for algorithm in algorithms:
    for frequency in range(1, 2):
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
            y.append(-position[1])
            z.append(position[2] / 30)

        # GROUND TRUTH - Square, triangle, spiral, etc.
        ground_truth = [[0, 0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16, 0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0], [ 0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16, 0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0, 0]]
        
        # ERROR MEASUREMENT
        error.calculate(ground_truth[0],ground_truth[1], x, y)

        # PLOTS
        # 2D PLOTTING
        plt.plot(x, y)
        plt.plot(ground_truth[0],ground_truth[1])
        plt.suptitle('Madgwick AHRS')
        plt.show()
        # plt.savefig('figuresMadgwick/figure' + str(frequency))

        # # 3D PLOTTING
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection="3d")
        # ax.scatter(x, y, z)
        # plt.show()
        # # plt.savefig('figuresMadgwick/figure' + str(frequency))

        # # 2D VIDEO PLOTTING
        # fig = plt.figure()
        # # creating a subplot
        # ax = fig.add_subplot(1, 1, 1)
        # xs = []
        # ys = []

        # def animate(i):
        #     xs.append(x[i])
        #     ys.append(y[i])
        #     ax.clear()
        #     ax.plot(xs, ys)

        # ani = animation.FuncAnimation(fig, animate, interval=1, frames=5000)
        # plt.show()
        # # ani.save('2Dplot.gif')

        # # 3D VIDEO PLOTTING
        # fig = plt.figure()
        # # creating a subplot
        # ax = fig.add_subplot(111, projection='3d')

        # xs = []
        # ys = []
        # zs = []

        # def animate(i):
        #     xs.append(x[i])
        #     ys.append(y[i])
        #     zs.append(z[i])

        #     ax.clear()
        #     ax.scatter(xs, ys, zs)
        #     # ax.plot(xs, ys, zs)
        #     ax.view_init(30, i/3)

        # ani = animation.FuncAnimation(fig, animate, interval=1, frames=5000)

        # plt.show()
        # # ani.save('2Dplot.gif')
