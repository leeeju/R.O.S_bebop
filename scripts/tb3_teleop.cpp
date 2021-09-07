#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>

#include <iostream>
#include <unistd.h>
#include <termios.h>

#define MAX_LIN_SPD     0.220
#define MAX_ANG_SPD     2.840

#define MIN_LIN_SPD    -0.220
#define MIN_ANG_SPD    -2.840

#define LIN_STP         0.011
#define ANG_STP         0.142

int getch(void);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tb3_teleop");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist tw;
    ros::Rate loop_rate(10);
    
    int ch = ' ';  double lin_speed_x = 0;  double ang_speed_z = 0;
    
    printf("type 'w', 's', 'a', 'd', ' ' to control tb3. ('^C' for quit)\n");
    
    while(ros::ok()) {
    
        ch = getch();
        
        if     (ch == 'w') {
            if(lin_speed_x + LIN_STP <= MAX_LIN_SPD)
                lin_speed_x = lin_speed_x + LIN_STP;
            else
                lin_speed_x = MAX_LIN_SPD;
        }
        
        else if(ch == 's') {
            if(MIN_LIN_SPD <= lin_speed_x - LIN_STP)
                lin_speed_x = lin_speed_x - LIN_STP;
            else
                lin_speed_x = MIN_LIN_SPD;
        }
        
        else if(ch == 'a') {
            if(MAX_ANG_SPD >= ang_speed_z + ANG_STP)
                ang_speed_z = ang_speed_z + ANG_STP;
            else
                ang_speed_z = MAX_ANG_SPD;
        }
        
        else if(ch == 'd') {
            if(MIN_ANG_SPD <= ang_speed_z - ANG_STP)
                ang_speed_z = ang_speed_z - ANG_STP;
            else
                ang_speed_z = MIN_ANG_SPD;
        }
        
        else if(ch == ' ') {
            lin_speed_x = 0.0;  ang_speed_z = 0.0;
        }
        
        else if(ch == '\x03')   // Ctrl-C
            break;
        
        else;
        
        tw.linear.x  = lin_speed_x;
        tw.angular.z = ang_speed_z;
        
        printf("linear velocity = %lf,\t angular velocity = %lf\n", tw.linear.x, tw.angular.z);
        
        pub.publish(tw);
        loop_rate.sleep();
    }
    return 0;
}

int getch()
{
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}
