import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches
import error

def plot2D(ground_truth_x, ground_truth_y, estimated_x, estimated_y, turn_x, turn_y):
    """Plots Ground Truth and Estimated Position in 2D"""
    plt.scatter(estimated_x, estimated_y)
    # plt.scatter(turn_x, turn_y, s=30, c='r')
    plt.plot(ground_truth_x, ground_truth_y)
    # plt.suptitle(str(turn_error) + ' ' + str(experiment_error))
    plt.show()
    # plt.savefig('figuresMadgwick/figure' + str(frequency))
    plt.clf()

def plot3D(ground_truth_x, ground_truth_y, ground_truth_z, estimated_x, estimated_y, estimated_z, turn_x, turn_y):
    """Plots Ground Truth and Estimated Position in 3D"""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(estimated_x, estimated_y, estimated_z)
    ax.plot(ground_truth_x,ground_truth_y,ground_truth_z)
    plt.show()
    # plt.savefig('figuresMadgwick/figure' + str(frequency))

def video2D(x, y):
    # 2D VIDEO PLOTTING
    fig = plt.figure()
    # creating a subplot
    ax = fig.add_subplot(1, 1, 1)
    xs = []
    ys = []

    def animate(i):
        xs.append(x[i])
        ys.append(y[i])
        ax.clear()
        ax.plot(xs, ys)

    ani = animation.FuncAnimation(fig, animate, interval=1, frames=5000)
    plt.show()
    # ani.save('2Dplot.gif')

def video3D(x, y, z):
    # 3D VIDEO PLOTTING
    fig = plt.figure()
    # creating a subplot
    ax = fig.add_subplot(111, projection='3d')

    xs = []
    ys = []
    zs = []

    def animate(i):
        xs.append(x[i])
        ys.append(y[i])
        zs.append(z[i])

        ax.clear()
        ax.scatter(xs, ys, zs)
        # ax.plot(xs, ys, zs)
        ax.view_init(30, i/3)

    ani = animation.FuncAnimation(fig, animate, interval=1, frames=5000)

    plt.show()
    # ani.save('2Dplot.gif')
