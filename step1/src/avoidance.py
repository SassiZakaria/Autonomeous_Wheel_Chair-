#!/usr/bin/python3

# import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist

# angY = 0.0
# angZ = 0.0
# linX = 0.0



# def laserScanCallback(laser):
#     global angY, angZ, linX

#     bol_str8 = True
  
#     range1=laser.ranges[0:359]
#     range2=laser.ranges[359:719]
#     # right detected, turn left
#     for range in range2:
#         if 0.1< range < 1:
#             rospy.loginfo("TURN LEFT")
#             linX = 0.0
#             angY = 1.0
#             angZ = -1.0
#             bol_str8 = False

#     # left detected, turn right
#     for range in range1:
#         if 0.1< range < 1:
#             rospy.loginfo("TURN RIGHT")
#             linX = 0.0
#             angY = 1.0
#             angZ = 1.0
#             bol_str8 = False

#     # nothing detected, go straight
#     if bol_str8:
#         rospy.loginfo("GO STRAIGHT")
#         linX = 0.3
#         angY = 0.0
#         angZ = 0.0


# if __name__ == "__main__":
#     rospy.init_node("hLaserReader")
#     hokuyoSubscriber = rospy.Subscriber("/m2wr/laser/scan", LaserScan, laserScanCallback)
#     pubVelocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
#     rate = rospy.Rate(10)
#     linX = 0.0
#     angY = 0.0
#     angZ = 0.0
#     while not rospy.is_shutdown():
#         msg = Twist()
#         msg.linear.x = linX
#         msg.angular.y = angY
#         msg.angular.z = angZ

#         pubVelocity.publish(msg)
#         rate.sleep()

#     rospy.spin()






import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

angY = 0.0
angZ = 0.0
linX = 0.0



def laserScanCallback(laser):
    global angY, angZ, linX

    bol_str8 = True
  

    # right detected, turn left
    for range in [laser.ranges[0], laser.ranges[1], laser.ranges[2], laser.ranges[3], laser.ranges[4]]:
        if 0.2 < range < 1:
           
            rospy.loginfo("TURN RIGHT")
            linX = 0.0
            angY = 1.0
            angZ = 1.0
            bol_str8 = False

           
           
    # left detected, turn right
    for range in [laser.ranges[5], laser.ranges[6], laser.ranges[7], laser.ranges[8], laser.ranges[9]]:
        if 0.2 < range < 1:
        
            rospy.loginfo("TURN LEFT")
            linX = 0.0
            angY = 1.0
            angZ =  -1.0
            bol_str8 = False    

        # nothing detected, go straight
    if bol_str8:
        rospy.loginfo("GO STRAIGHT")
        linX = 0.3
        angY = 0.0
        angZ = 0.0


if __name__ == "__main__":
    rospy.init_node("hLaserReader")
    hokuyoSubscriber = rospy.Subscriber("/m2wr/laser/scan", LaserScan, laserScanCallback)
    pubVelocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    linX = 0.0
    angY = 0.0
    angZ = 0.0
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = linX
        msg.angular.y = angY
        msg.angular.z = angZ

        pubVelocity.publish(msg)
        rate.sleep()

    rospy.spin()
