#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import cos, sin, atan
from numpy import sign
import time

delta_t_w = 2
delta_t_v = 10
is_first_step_completed = False
is_second_step_completed = False
goals = []  # List to store the goals

def odom_callback(odom):
    ask_and_generate(odom, CommandPublisher, goal)

def ask_and_generate(odom, CommandPublisher, goal):
    global is_first_step_completed

    x_0 = odom.pose.pose.position.x
    y_0 = odom.pose.pose.position.y

    orientation_quat = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )
    roll, pitch, yaw = euler_from_quaternion(orientation_quat)
    theta_0 = yaw
    l=sign((goal[1] - y_0))
    theta_g1 = atan((goal[1] - y_0) / (goal[0] - x_0))
    if (goal[0]-x_0)<0.2:
        w1 = (l*theta_g1 - theta_0) / delta_t_w
    else:
        w1 = (theta_g1 - theta_0) / delta_t_w
    if x_0 == goal[0]:
        v = abs((goal[1] - y_0) / (delta_t_v * sin(theta_g1)))
    else:
        v = abs((goal[0] - x_0) / (delta_t_v * cos(theta_g1)))
    # 1st step
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = w1
    CommandPublisher.publish(msg)

    time.sleep(delta_t_w)

    msg.linear.x = 0.0
    msg.angular.z = 0.0
    CommandPublisher.publish(msg)

    time.sleep(2)

    # 2nd step 
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = 0.0
    CommandPublisher.publish(msg)

    time.sleep(delta_t_v)

    msg.linear.x = 0.0
    msg.angular.z = 0.0
    CommandPublisher.publish(msg)

    is_first_step_completed = True

def odom_callback2(odom):
    ask_and_generate2(odom, CommandPublisher, goal)

def ask_and_generate2(odom, CommandPublisher, goal):
    global is_second_step_completed

    if is_first_step_completed and not is_second_step_completed:
        # 3rd step 
        orientation_quat2 = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(orientation_quat2)
        theta_02 = yaw


        w2 = (goal[2] - theta_02) / delta_t_w
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = w2
        CommandPublisher.publish(msg)

        time.sleep(delta_t_w)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        CommandPublisher.publish(msg)

        is_second_step_completed = True

if __name__ == "__main__":
    rospy.init_node("inverse_kinematics")
    CommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    num_goals = int(input("Enter the number of goals: "))
    for _ in range(num_goals):
        x_g = float(input("Enter the goal x-coordinate: "))
        y_g = float(input("Enter the goal y-coordinate: "))
        theta_g = float(input("Enter the goal orientation: "))
        goals.append((x_g, y_g, theta_g))
    for goal in goals:
        x_g, y_g, theta_g = goal
        is_first_step_completed = False
        is_second_step_completed = False

        OdomSubscriber = rospy.Subscriber("/odom", Odometry, odom_callback)
        while not is_first_step_completed:
            # Waiting for the first step to complete
            pass
        OdomSubscriber.unregister()  # Unsubscribe from "/odom" topic

        OdomSubscriber2 = rospy.Subscriber("/odom", Odometry, odom_callback2)
        while not is_second_step_completed:
            # Waiting for the second step to complete
            pass
        OdomSubscriber2.unregister()  # Unsubscribe from "/odom" topic
    if is_first_step_completed and is_second_step_completed:
            rospy.signal_shutdown("Movement completed.")
    rospy.spin()
