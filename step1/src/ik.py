#!/usr/bin/python3

import rospy 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import cos, sin, atan
import time

def ask_and_generate(odom, CommandPublisher, x_g, y_g):
    delta_t_w = 2
    delta_t_v = 2

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
    theta_g = atan((y_g-y_0) / (x_g-x_0))
    w = theta_g / delta_t_w

    if x_0 == x_g:
        v = (y_g - y_0) / (delta_t_v * sin(theta_g))
    else:
        v = (x_g - x_0) / (delta_t_v * cos(theta_g))
        
    
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = w
    CommandPublisher.publish(msg)

    time.sleep(delta_t_w)

    # Stop the robot
    msg.linear.x = 0
    msg.angular.z = 0
    CommandPublisher.publish(msg)


    time.sleep(2)

    msg = Twist()
    msg.linear.x= v
    msg.angular.z = 0
    CommandPublisher.publish(msg)

    time.sleep(delta_t_v)

    # Stop the robot
    msg.linear.x = 0
    msg.angular.z = 0
    CommandPublisher.publish(msg)

    # Shutdown the node after completing the movement
    rospy.signal_shutdown("Movement completed.")


if __name__ == "__main__":
    rospy.init_node("inverse_kinematics")
    CommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    x_g = float(input("Enter the goal x-coordinate: "))
    y_g = float(input("Enter the goal y-coordinate: "))

    def odom_callback(odom):
        ask_and_generate(odom, CommandPublisher, x_g, y_g)

    OdomSubscriber = rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.spin()
