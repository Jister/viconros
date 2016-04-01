#include "ros/ros.h"
#include "viconros/viconmocap.h"

#define 
void viconCallback(const viconros::viconmocap::ConstPtr& msg)
{
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/vicon", 10, viconCallback);

  ros::spin();

  return 0;
}