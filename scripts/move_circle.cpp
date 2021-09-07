#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_circle");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    geometry_msgs::Twist tw;
    ros::Rate loop_rate(10);
    
    tw.linear.x  = 0.5; // (m/sec)
    tw.angular.z = 0.5; // (rad/sec)

    while(ros::ok()){
        pub.publish(tw);
        loop_rate.sleep();
    }
    return 0;
}
