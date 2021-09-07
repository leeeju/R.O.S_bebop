#include "ros/ros.h"
#include "std_msgs/String.h"

void cb_func(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_hello");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("hello", 10, cb_func);
  ros::spin();

  return 0;
}
