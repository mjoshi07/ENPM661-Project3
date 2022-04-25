import numpy as np

import Config as cf


def out_of_bounds(position):
    # ORIGIN - left bottom corner as given in project ppt

    x, y = position[0], position[1]

    outside_left_boundary = x < 0
    outside_right_boundary = x > cf.MAP_SIZE
    outside_top_boundary = y > cf.MAP_SIZE
    outside_bottom_boundary = y < 0

    if outside_left_boundary or outside_top_boundary or outside_right_boundary or outside_bottom_boundary:
        return True

    return False


def inside_obstacle_space(position, clearance):
    # ORIGIN - left bottom corner as given in project ppt

    x, y = position[0], position[1]

    inside_left_boundary = x < clearance
    if inside_left_boundary:
         return True

    inside_top_boundary = y > cf.MAP_SIZE - clearance
    if inside_top_boundary:
         return True

    inside_right_boundary = x > cf.MAP_SIZE - clearance
    if inside_right_boundary:
         return True

    inside_bottom_boundary = y < clearance
    if inside_bottom_boundary:
         return True

    # bottom left circle
    inside_obstacle_1 = (np.square(x - cf.OBJ_1_CENTER_X) + np.square(y - cf.OBJ_1_CENTER_Y)) \
                        < np.square(cf.OBJ_1_RADIUS + clearance)
    if inside_obstacle_1:
         return True

    # middle left square
    inside_obstacle_2 = (cf.OBJ_2_CENTER_X + (cf.OBJ_2_WIDTH / 2) + clearance) > x > \
                        (cf.OBJ_2_CENTER_X - (cf.OBJ_2_WIDTH / 2) - clearance) and \
                        (cf.OBJ_2_CENTER_Y - (cf.OBJ_2_HEIGHT / 2) - clearance) < y < \
                        (cf.OBJ_2_CENTER_Y + (cf.OBJ_2_HEIGHT / 2) + clearance)
    if inside_obstacle_2:
         return True

    # top left circle
    inside_obstacle_3 = (np.square(x - cf.OBJ_3_CENTER_X) + np.square(y - cf.OBJ_3_CENTER_Y)) \
                        < np.square(cf.OBJ_3_RADIUS + clearance)
    if inside_obstacle_3:
         return True

    # middle rectangle
    inside_obstacle_4 = (cf.OBJ_4_CENTER_X + (cf.OBJ_4_WIDTH / 2) + clearance) > x > \
                        (cf.OBJ_4_CENTER_X - (cf.OBJ_4_WIDTH / 2) - clearance) and \
                        (cf.OBJ_4_CENTER_Y - (cf.OBJ_4_HEIGHT / 2) - clearance) < y < \
                        (cf.OBJ_4_CENTER_Y + (cf.OBJ_4_HEIGHT / 2) + clearance)
    if inside_obstacle_4:
        return True

    # bottom right rectangle
    inside_obstacle_5 = (cf.OBJ_5_CENTER_X - (cf.OBJ_5_WIDTH / 2) - clearance) < x < \
                        (cf.OBJ_5_CENTER_X + (cf.OBJ_5_WIDTH / 2) + clearance) and \
                        (cf.OBJ_5_CENTER_Y - (cf.OBJ_5_HEIGHT / 2) - clearance) < y < \
                        (cf.OBJ_5_CENTER_Y + (cf.OBJ_5_HEIGHT / 2) + clearance)
    if inside_obstacle_5:
         return True

    return False


def calculate_euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def reached_goal(cur_position, goal_position):

    if calculate_euclidean_distance(cur_position, goal_position) <= cf.GOAL_THRESH:
        return True

    return False


def handle_theta_edge(curr_theta):
    final_angle = curr_theta
    while final_angle < 0:
        final_angle += 360
    while final_angle >= 360:
        final_angle -= 360

    return final_angle