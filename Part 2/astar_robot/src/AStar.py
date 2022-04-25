
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import os
import Config as cf
import Solver as sv



def get_command(initial_theta, action):
    robot_radius = cf.ROBOT_WHEEL_RADIUS
    robot_len = cf.ROBOT_WHEEL_BASE

    vel_left, vel_right = action[0]*2*np.pi/60, action[1]*2*np.pi/60
    ang_vel = (robot_radius/robot_len) * (vel_right-vel_left) + np.pi*initial_theta/180
    velx = (robot_radius/2)*(vel_left+vel_right)*np.cos(ang_vel)
    vely = (robot_radius/2)*(vel_left+vel_right)*np.sin(ang_vel)
    lin_vel = np.sqrt(velx**2 + vely**2)

    return lin_vel, ang_vel


def move_turtleBot(path):
    count=0

    initial_th = 0
    rate = rospy.Rate(10)

    for node in path:
        coords, action = node
        t = 0
        while not rospy.is_shutdown():
            if t<cf.TIME_DURATION:
                lin_vel, ang_vel = get_command(initial_th, action)
                msg.linear.x = lin_vel*10
                msg.angular.z = ang_vel*10
                vel_pub.publish(msg)
                t+=cf.DELTA_T
                rate.sleep()
            else:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0

                vel_pub.publish(msg)
                t = 0
                break


if __name__ == "__main__":

    path, gz_startx, gz_start_y, gz_start_theta = sv.solve()
    print("Open new terminal and execute following command: \n\nsource devel/setup.bash && roslaunch astar_robot astar.launch x_pos:={} y_pos:={} yaw:={}".format(gz_startx, gz_start_y, gz_start_theta))
    print("\n\nPress any key after turtlebot is spawned and press enter")
    a = input()
    print("\n\n")
    rospy.init_node('Astar', anonymous=True)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    msg = Twist()


    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    vel_pub.publish(msg)

    move_turtleBot(path)
