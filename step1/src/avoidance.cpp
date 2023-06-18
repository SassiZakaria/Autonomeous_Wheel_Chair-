#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

float angY, angZ, linX;

void laserScanCallback(const sensor_msgs::LaserScan laser)
{
    bool bol_str8=true;
    
    //right detected, turn left
    for (const auto range : {laser.ranges[0], laser.ranges[1], laser.ranges[2], laser.ranges[3], laser.ranges[4],})
    {
    if ( range<3 && range>1 )
    {
        ROS_INFO("TURN LEFT");
        linX = 0.0f;
        angY = 1.0f;
        angZ = -1.0f;
        bol_str8=false;
    }
    }

    //left detected, turn right
    for (const auto range : {laser.ranges[5], laser.ranges[6], laser.ranges[7], laser.ranges[8], laser.ranges[9],})
    {
      if( range<3 && range>1 )
      {
        ROS_INFO("TURN RIGHT");
        linX = 0.0f;
        angY = 1.0f;
        angZ = 1.0f;
        bol_str8=false;
      }
    }

    //nothing detected, go straight
    if (bol_str8==true)
    {
        ROS_INFO("GO STRAIGHT");
        linX = 0.3f;
        angY = 0.0f;
        angZ = 0.0f;
    }

}
  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hLaserReader");
  ros::NodeHandle nodeHandler;
  ros::Subscriber hokuyoSubscriber = nodeHandler.subscribe("/m2wr/laser/scan", 1, laserScanCallback);
  ros::Publisher pubVelocity = nodeHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ros::Rate rate(10);
  linX = 0.0f;
  angY = 0.0f;
  angZ = 0.0f;
  while (ros::ok()) {
    geometry_msgs::Twist msg;
    msg.linear.x = linX;
    msg.angular.y = angY;
    msg.angular.z = angZ;

    pubVelocity.publish(msg);
    rate.sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}



















