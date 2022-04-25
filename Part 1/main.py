import math
import time
import numpy as np
from queue import PriorityQueue

import Node as nd
import Utils as ut
import Config as cf
import Visualize as vz


def move_robot(cur_position, action, clearance):
    x_i, y_i, theta_i = cur_position[0], cur_position[1], cur_position[2]
    x_n = x_i
    y_n = y_i
    theta_n = np.radians(theta_i)

    ul, ur = action[0], action[1]

    t = 0.0
    cost_to_come = 0.0
    while t < cf.TIME_DURATION:
        t += cf.DELTA_T
        x_s = x_n
        y_s = y_n
        x_n += (0.5 * cf.ROBOT_WHEEL_RADIUS) * (ul + ur) * np.cos(theta_n) * cf.DELTA_T
        y_n += (0.5 * cf.ROBOT_WHEEL_RADIUS) * (ul + ur) * np.sin(theta_n) * cf.DELTA_T
        if ut.inside_obstacle_space((x_n, y_n), clearance):
            return None
        if ut.out_of_bounds((x_n, y_n)):
            return None
        theta_n += (cf.ROBOT_WHEEL_RADIUS / cf.ROBOT_WHEEL_BASE) * (ur - ul) * cf.DELTA_T
        cost_to_come += ut.calculate_euclidean_distance((x_s, y_s), (x_n, y_n))

    theta_n = np.degrees(theta_n)
    next_position = (x_n, y_n, theta_n)
    return [next_position, cost_to_come]


def run_A_star(start, goal, action_set, clearance):

    visited_nodes = np.ones((cf.SIZE, cf.SIZE, cf.N_A)) * math.inf

    pq = PriorityQueue()
    inital_node = nd.Node(start, None, 0)
    pq.put([inital_node.cost_to_come, inital_node.node_state, inital_node])

    while not pq.empty():
        # pop a node
        cur_node = pq.get()[2]

        # check if reached goal position
        if ut.reached_goal(cur_node.node_state, goal):
            print("[INFO]: Reached Goal Node")
            return cur_node

        for action in action_set:
            next_node = move_robot(cur_node.node_state, action, clearance)
            if next_node is not None:
                next_position = next_node[0]
                cost_to_come = next_node[1]
                t = ut.handle_theta_edge(next_position[2])
                next_position = (next_position[0], next_position[1], t)
                node = nd.Node(next_position, cur_node, cost_to_come, action=action)

                x = int((round(next_position[0] / cf.THRESH_DIST) * cf.THRESH_DIST) * cf.N_D)
                y = int((round(next_position[1] / cf.THRESH_DIST) * cf.THRESH_DIST) * cf.N_D)
                t = round(t / cf.THRESH_ANGLE)
                theta = int(t)
                dist = ut.calculate_euclidean_distance(node.node_state, goal)

                if visited_nodes[x, y, theta] > node.cost_to_come + dist:
                    visited_nodes[x, y, theta] = node.cost_to_come + dist
                    pq.put([node.cost_to_come + dist, node.node_state, node])


if __name__ == "__main__":

    print("Enter Clearance: ")
    clearance = float(input())
    print("Enter RPM1: ")
    rpm1 = float(input())
    print("Enter RPM2: ")
    rpm2 = float(input())

    flag1 = True
    flag2 = True
    while flag1 or flag2:
        print("Enter START position")
        print("X: ")
        start_x = float(input())
        print("Y: ")
        start_y = float(input())
        print("Theta in Degrees: ")
        start_theta = float(input())

        start_position = (start_x, start_y, start_theta)
        # boundary check
        if ut.out_of_bounds(start_position):
            print("[ERROR]: Start Position out of map!!!")
            flag1 = True
        else:
            flag1 = False
        if ut.inside_obstacle_space(start_position, clearance):
            print("[ERROR]: Start Position inside of obstacle space!!!")
            flag2 = True
        else:
            flag2 = False

    flag1 = True
    flag2 = True
    while flag1 or flag2:
        print("Enter GOAL position")
        print("X: ")
        goal_x = float(input())
        print("Y: ")
        goal_y = float(input())

        goal_position = (goal_x, goal_y, 0)
        # obstacle space check
        if ut.inside_obstacle_space(goal_position, clearance):
            print("[ERROR]: Goal Position inside of obstacle space!!!")
            flag1 = True
        else:
            flag1 = False

        if ut.out_of_bounds(goal_position):
            print("[ERROR]: Goal Position out of map!!!")
            flag2 = True
        else:
            flag2 = False

    action_set = nd.get_action_set(rpm1, rpm2)

    print("[INFO]: Started A-star")

    # no errors in start or goal positions, start A* algorithm
    st = time.time()
    final_node = run_A_star(start_position, goal_position, action_set, clearance)
    end = time.time()
    print("[INFO]: Search complete in {} seconds".format(end - st))
    path = nd.backtrack(final_node)

    vz.plot_my_path(start_position, path, goal_position, action_set)
