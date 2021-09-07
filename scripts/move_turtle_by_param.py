#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist
        
def move_turtle():
    pb = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=20)
    tw = Twist()
    tw.linear.x  = 1.00
    tw.angular.z = 0.50 
    tw.angular.y = 0.80       
    pb.publish(tw)
    
if __name__ == '__main__':
    try:
        rospy.init_node('move_by_param')

        while not rospy.is_shutdown():
            param = rospy.get_param("/turtle1/go_turtle")
            
            if param is True:
                move_turtle()                
            else:	pass

    except rospy.ROSInterruptException:   pass
    
