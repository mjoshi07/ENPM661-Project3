import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import Config as cf


def plot_curve(x_i, y_i, theta_i, ul, ur, color, line_thickness,draw=True):
    t = 0.0
    x_n = x_i
    y_n = y_i
    theta_n = np.radians(theta_i)
    dt = cf.DELTA_T
    while t < cf.TIME_DURATION:
        t = t + dt
        x_s = x_n
        y_s = y_n
        x_n += 0.5 * cf.ROBOT_WHEEL_RADIUS * (ul + ur) * np.cos(theta_n) * dt
        y_n += 0.5 * cf.ROBOT_WHEEL_RADIUS * (ul + ur) * np.sin(theta_n) * dt
        theta_n += (cf.ROBOT_WHEEL_RADIUS / cf.ROBOT_WHEEL_BASE) * (ur - ul) * dt
        if draw:
            plt.plot([x_s, x_n], [y_s, y_n], color=color, linewidth=line_thickness)

    theta_n = np.degrees(theta_n)

    return x_n, y_n, theta_n


def plot_my_path(start_position, path, goal_position, action_set):
    sx, sy, sz = start_position[0], start_position[1], start_position[2]
    gx, gy = goal_position[0], goal_position[1]
    fig, ax = plt.subplots()
    plt.title('A* path planning with non-holonomic constraints', fontsize=20)
    ax.set(xlim=(0, 10), ylim=(0, 10))

    x = (round(start_position[0] / cf.THRESH_DIST) * cf.THRESH_DIST)
    y = (round(start_position[1] / cf.THRESH_DIST) * cf.THRESH_DIST)

    plt.plot(x, y, color='green', marker='o', linestyle='dashed', linewidth=1, markersize=4)
    plt.plot(sx, sy, color='green', marker='o', linestyle='solid', linewidth=2,
             markersize=4)
    plt.plot(gx, gy, color='blue', marker='o', linestyle='solid', linewidth=2,
             markersize=8)

    c1 = plt.Circle((cf.OBJ_1_CENTER_X, cf.OBJ_1_CENTER_Y), cf.OBJ_1_RADIUS, fill=None)
    c2 = plt.Circle((cf.OBJ_3_CENTER_X, cf.OBJ_3_CENTER_Y), cf.OBJ_3_RADIUS, fill=None)
    currentAxis = plt.gca()
    currentAxis.add_patch(Rectangle((cf.OBJ_2_CENTER_X - cf.OBJ_2_WIDTH / 2, cf.OBJ_2_CENTER_Y - cf.OBJ_2_HEIGHT / 2),
                                    cf.OBJ_2_WIDTH, cf.OBJ_2_HEIGHT, fill=None, alpha=1))
    currentAxis.add_patch(Rectangle((cf.OBJ_4_CENTER_X - cf.OBJ_4_WIDTH / 2, cf.OBJ_4_CENTER_Y - cf.OBJ_4_HEIGHT / 2),
                                    cf.OBJ_4_WIDTH, cf.OBJ_4_HEIGHT, fill=None, alpha=1))
    currentAxis.add_patch(Rectangle((cf.OBJ_5_CENTER_X - cf.OBJ_5_WIDTH / 2, cf.OBJ_5_CENTER_Y - cf.OBJ_5_HEIGHT / 2),
                                    cf.OBJ_5_WIDTH, cf.OBJ_5_HEIGHT, fill=None, alpha=1))

    ax.add_artist(c1)
    ax.add_artist(c2)
    ax.set_aspect('equal')
    plt.grid()

    # plt.pause(1)
    # for action in path:
    #     for act in action_set:
    #         x2 = plot_curve(sx, sy, sz, act[0], act[1], (0, 0, 1), 0.5)
    #         # plt.pause(0.01)
    #     x1 = plot_curve(sx, sy, sz, action[1][0], action[1][1], (1, 0, 0), 1, False)
    #     sx = x1[0]
    #     sy = x1[1]
    #     sz = x1[2]
    #     # plt.pause(0.01)

    sx, sy, sz = start_position[0], start_position[1], start_position[2]
    for action in path:
        x1 = plot_curve(sx, sy, sz, action[1][0], action[1][1], (1, 0, 0), 2)
        sx = x1[0]
        sy = x1[1]
        sz = x1[2]
        # plt.pause(0.001)
    print("[INFO]: Reached Goal")
    plt.show()
    plt.pause(1)
    plt.close()