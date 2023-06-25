#!/usr/bin/python3

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laserScanCallback(laser):
    global angY, angZ, linX

    if (0.2 < laser.ranges[2] < 0.5) and (0.2 < laser.ranges[1] < 0.7) and (0.2 < laser.ranges[0] < 0.5):
        rospy.loginfo("TURN RIGHT")
        linX = 0.0
        angZ = 1.0
    elif (0.2 < laser.ranges[2] < 0.5) and (0.2 < laser.ranges[1] < 0.7) and not (0.2 < laser.ranges[0] < 0.5):
        rospy.loginfo("TURN LEFT")
        linX = 0.0
        angZ = -1.0
    elif (0.2 < laser.ranges[2] < 0.5) and not (0.2 < laser.ranges[1] < 0.7):
        rospy.loginfo("GO STRAIGHT")
        linX = 0.3
        angY = 0.0
        angZ = 0.0
    elif not (0.2 < laser.ranges[2] < 0.5) and (0.2 < laser.ranges[1] < 0.7):
        rospy.loginfo("TURN RIGHT")
        linX = 0.0
        angZ = 1.0
    elif not (0.2 < laser.ranges[2] < 0.5) and not (0.2 < laser.ranges[1] < 0.7) and (0.2 < laser.ranges[0] < 0.5):
        rospy.loginfo("TURN RIGHT")
        linX = 0.0
        angZ = 1.0
    elif not (0.2 < laser.ranges[2] < 0.5) and not (0.2 < laser.ranges[1] < 0.7) and not (0.2 < laser.ranges[0] < 0.5):
        rospy.loginfo("GO STRAIGHT")
        linX = 0.3
        angY = 0.0
        angZ = 0.0
    elif (0.2 < laser.ranges[2] < 1) and (0.2 < laser.ranges[1] < 0.7) and (0.2 < laser.ranges[0] < 1):
        rospy.loginfo("TURN TURN")
        linX = 0.0
        angZ = -1.0
        time.sleep(1)

    # Publish velocity
    msg = Twist()
    msg.linear.x = linX
    msg.angular.y = angY
    msg.angular.z = angZ
    pubVelocity.publish(msg)

if __name__ == "__main__":
    rospy.init_node("hLaserReader")
    pubVelocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/m2wr/laser/scan", LaserScan, laserScanCallback)
    rate = rospy.Rate(10)
    linX = 0.0
    angY = 0.0
    angZ = 0.0
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()