#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import cos, sin, atan
from numpy import sign
import time

class Goal:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class RobotController:
    def __init__(self):
        rospy.init_node("inverse_kinematics")
        self.CommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.OdomSubscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.goals = []
        self.current_goal_index = 0

    def odom_callback(self, odom):
        if self.current_goal_index < len(self.goals):
            self.ask_and_generate(odom, self.goals[self.current_goal_index])
            self.current_goal_index += 1

            # Unsubscribe from /odom topic after reaching a goal
            self.OdomSubscriber.unregister()

            # Subscribe to /odom topic again if there are more goals
            if self.current_goal_index < len(self.goals):
                self.OdomSubscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def ask_and_generate(self, odom, goal):
        delta_t_w = 3
        delta_t_v = 10

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

        # voire l'etat du yaw
        if yaw < 0:
            theta_0 = yaw
        else:
            theta_0 = -yaw

        if abs(goal.x - x_0) < 0.001:
            theta_g = 1.57
        else:
            if abs(goal.y - y_0) < 0.001:
                theta_g = 0
            else:
                theta_g = atan((goal.y - y_0) / (goal.x - x_0))

        w1 = (theta_g - theta_0) / delta_t_w

        print('theta_g=%f' % (theta_g * 180 / 3.14))

        w1 = (theta_g - theta_0) / delta_t_w
        print('w1 = %f' % w1)
        if theta_g == 1.57:
            v = (goal.y - y_0) / delta_t_v
        else:
            if (theta_g==0):
             v = (goal.x - x_0) / (delta_t_v * cos(theta_g))
            else:
                if abs(x_0 - goal.x) < 0.2:
                  v = (goal.y - y_0) / (delta_t_v * sin(theta_g))
                else:
                  v = (goal.x - x_0) / (delta_t_v * cos(theta_g))

        # voire la vitesse et recalculer
        if v < 0:
            theta_g = theta_g - 3.14

        print('theta_g=%f' % (theta_g * 180 / 3.14))
        w1 = (theta_g - theta_0) / delta_t_w
        print('w1 = %f' % w1)
        print('v = %f' % v)
        if theta_g == 1.57:
            v = (goal.y - y_0) / delta_t_v
        else:
            if (theta_g==0):
             v = (goal.x - x_0) / (delta_t_v * cos(theta_g))
            else:
                if abs(x_0 - goal.x) < 0.2:
                  v = (goal.y - y_0) / (delta_t_v * sin(theta_g))
                else:
                  v = (goal.x - x_0) / (delta_t_v * cos(theta_g))

        # 1st step
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = w1
        self.CommandPublisher.publish(msg)

        time.sleep(delta_t_w)

        msg.linear.x = 0
        msg.angular.z = 0
        self.CommandPublisher.publish(msg)

        time.sleep(0.5)

        # 2nd step
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = 0
        self.CommandPublisher.publish(msg)

        time.sleep(delta_t_v)

        msg.linear.x = 0
        msg.angular.z = 0
        self.CommandPublisher.publish(msg)

        # 3rd step
        orientation_quat2 = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(orientation_quat2)
        theta_02 = yaw
        w2 = (goal.theta - theta_02) / delta_t_w

        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = w2
        self.CommandPublisher.publish(msg)

        time.sleep(delta_t_w)

        msg.linear.x = 0
        msg.angular.z = 0
        self.CommandPublisher.publish(msg)

        rospy.loginfo("Position précédente x=%f, y=%f, theta=%f", x_0, y_0, yaw)

    def run(self):
        self.goals = [
            Goal(0, 13, -1.57),
            Goal(3.66, 7.78, -1.57),
            Goal(4.9, 2.16, -1.57),
            Goal(4.8, -4, -1.57),
            Goal(4.9, -8.6, -3.14),
            Goal(2.88, -8.6, -3),
            Goal(2.8, -9.7, -3),
            Goal(2.8, -10.92, -3),
        ]

        rospy.spin()

if __name__ == "__main__":
    controller = RobotController()
    controller.run()
