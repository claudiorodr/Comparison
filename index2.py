from squaternion import Quaternion
import numpy as np
from ahrs.filters import AngularRate, AQUA, Complementary, Davenport, EKF, FAMC, FLAE, Fourati, FQA, Madgwick, Mahony, OLEQ, QUEST, ROLEQ, SAAM, Tilt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches
import math
import integrations
import calculate

# Read data from files
data = np.loadtxt('data1.txt')
acc_data, rest = np.hsplit(data, [3])
acc_data *= 9.8
gyr_data, mag_data = np.hsplit(rest, [3])
gyr_data *= 0.017453
mag_data /= 1000
# Sampling rate
deltat = 0.1  # 10Hz

algorithms = [EKF, Madgwick, Mahony]

for algorithm in algorithms:
    for frequency in range(1, 21):
        dist_data = []
        # Saving ahrs estimations to array
        estimation = []
        # Start ahrs estimation
        if algorithm == FAMC or algorithm == FLAE or algorithm == FQA or algorithm == OLEQ or algorithm == QUEST or algorithm == SAAM or algorithm == Tilt:
            ahrs = algorithm(acc=acc_data, mag=mag_data)  # Returns Quaternions
        else:
            ahrs = algorithm(acc=acc_data, gyr=gyr_data, mag=mag_data,
                             frequency=frequency)  # Returns Quaternions

        for t in range(1, len(data)):
            # Integrate distance from X-axis acceleration
            positX, distanceX = calculate.distance(
                acc_data[t-1][0]/9.8, acc_data[t-1][1]/9.8, acc_data[t-1][2]/9.8, deltat)
            dist_data.append(distanceX)
            q = Quaternion(ahrs.Q[t][0], ahrs.Q[t][1],
                           ahrs.Q[t][2], ahrs.Q[t][3])
            estimation.append(q.to_euler(degrees=True))

        # Convert estimation to numpy array
        estimation = np.array(estimation)
        positions = []
        for index, distance in enumerate(dist_data, start=-1):
            # Calculate X, Y, Z positions - Distance * heading angle. Update the coordinate with each function
            coordinateX, coordinateY, coordinateZ = integrations.getCoordinate()
            positionX = distance * \
                math.cos(math.radians(estimation[index][2])) + coordinateX
            positionY = distance * \
                math.sin(math.radians(estimation[index][2])) + coordinateY
            positionZ = distance * \
                math.sin(math.radians(estimation[index][0])) + coordinateZ
            positions.append([positionX, positionY, positionZ])
            integrations.updateCoordinates(positionX, positionY, positionZ)

        integrations.updateCoordinates(0, 0, 0)
        x, y, z = [], [], []

        for position in positions:
            x.append(position[0]/2)
            y.append(position[1]/2)
            z.append(position[2]/30)

        # PLOTS

        # 2D PLOTTING
        plt.scatter(x, y)
        plt.plot([0, 0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16, 0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0], [
                 0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16, 0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0, 0])
        plt.suptitle('Madgwick AHRS')
        # plt.show()
        plt.savefig('figuresMadgwick/figure' + str(frequency))
        plt.clf()

        # # 3D PLOTTING
        # fig = plt.figure()
        # plt.plot([0, 0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16, 0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0],
        #          [0, 4, 4, 0, 0, 8, 8, 0, 0, 12, 12, 0, 0, 16, 16,
        #              0, 0, 20, 20, 0, 0, 24, 24, 0, 0, 28, 28, 0, 0],
        #          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(x, y, z)
        # plt.show()
        # plt.savefig('figuresMadgwick/figure' + str(frequency))

        # VIDEO PLOTTING
        # fig = plt.figure()
        # # creating a subplot
        # ax1 = fig.add_subplot(1, 1, 1)

        # xs = []
        # ys = []

        # def animate(i):
        #     xs.append(x[i])
        #     ys.append(y[i])

        #     ax1.clear()
        #     ax1.plot(xs, ys)

        # ani = animation.FuncAnimation(fig, animate, interval=1, frames=5000)

        # plt.show()
        # ani.save('2Dplot.gif')
