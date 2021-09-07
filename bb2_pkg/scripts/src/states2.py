#!/usr/bin/python
#-*- coding: utf-8 -*-


import rospy

from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged  # 비행 토픽
from bebop_msgs.msg import Ardrone3PilotingStateAlertStateChanged  # 상태 메시지 토픽
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  #베터리 상태 토픽


class State(object):
    def __init__(self, rotation_controller, location_controller):
        self.rotation_controller = rotation_controller
        self.location_controller = location_controller
        #print("Processing current state: {0}".format(self.__name__))

    def run(self):
        #pass
        print("State {0} is running".format(self.__name__))

    def next(self, event):
        #pass
        print("State {0} is interrupted by {1}".format(self.__name__, event.__name__))
# Search State
# Mode: Follow
# Conditions: Lost April Tag
# Actions: Turn in circles until April Tag is found.

                                 

class SearchState(State):   # 베터리
    def __init__(self, rotation_controller, location_controller):
        super(SearchState, self).__init__(rotation_controller,location_cotroller)
        self.rotation_controller.publish('search')
        self.locaiton_controller.publish('hover')

    def run(self):
        super(SearchState, self).run()

    def next(self, event):
        if event == 'tag found':
            rospy.loginfo("April Tag found, switching to follow state")
            return FollowState(self.rotation_controller, self.location_controller)
        elif event == 'manual':
            rospy.loginfo("Entering Manual Mode, switching to flying state")
            return FlyingState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            return CriticalState(self.rotation_controller, self.location_controller)
        else:
            return self

# Follow State
# Mode: Follow
# Conditions: April Tag in sight
# Actions: Keep April Tag at the center of the camera && Keep bebop a fixed distance away from the robot

class FollowState(State):   # 여기도 베터리
    def __init__(self, rotation_controller, location_controller):
        super(SearchState, self).__init__(rotation_controller,location_controller)
        self.rotation_controller.publish('follow')
        self.locaiton_controller.publish('follow')

    def run(self):
       super(SearchState, self).run()

    def next(self, event):
        if event == 'tag lost':
            rospy.loginfo("April Tag lost, switching to search state")
            return SearchState(self.rotation_controller, self.location_controller)
        elif event == 'manual':
            rospy.loginfo("Entering Manual Mode, switching to flying state")
            return FlyingState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            return CriticalState(self.rotation_controller, self.location_controller)
        else:
            return self

# Critical State
# Mode: Follow, Manual
# Conditions: Low Battery
# Actions: Land
class CriticalState(State):   # 여기도 베터리
    def __init__(self, rotation_controller, location_controller):
        rospy.logwarn("Landing")
        self.land = rospy.Publisher("land", Empty, queue_size=1)
        self.land.publish()
        super(CriticalState, self).__init__(rotation_controller, location_controller)

    def run(self):
        super(CriticalState, self).run()

    def next(self, event):
        if event == 'grounded':
            rospy.loginfo("Successfully landed, switching to Grounded State")
            return GroundedState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            self.land.publish() 
            return self
        else:
            return self

# Grounded State
# Mode: Follow, Manual
# Conditions: Motors off
# Actions: Nothing
class GroundedState(State):
    def __init__(self, rotation_controller, location_controller):
        super(GroundedState, self).__init__(rotation_controller, location_controller)

    def run(self):
        super(GroundedState, self).run()

    def next(self, event):
        if event == 'flying':
            rospy.loginfo("Successfully took off, switching to Flying State")
            return FlyingState(self.rotation_controller, self.location_controller)
        else:
            return self

# Flying State
# Mode: Follow
# Conditions: Motors on with operator control
# Actions: Obey Operator Control
def FlyingState(State):   # 여기 베터리
    def __init__(self, rotation_controller, location_controller):
        super(FlyingState,self).__init__(rotation_controller, location_controller)
      

    def run(self):
        super(FlyingState, self).run()

    def next(self, event):    # 베터리 없으면 수동 조종으로 변환  / 프린트 문을 넣서 잔량이 얼마인지도 출력 하게끔
        if event == 'grounded':
            rospy.loginfo("Landed, switching to grounded state")
            return GroundedState(self.rotation_controller, self.location_controller)
        elif event == 'follow':
            rospy.loginfo("switching to follow mode, switching to search state")
            return SearchState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            return CriticalState(self.rotation_controller, self.location_controller)
        else:
            return self
            
class Battery(State):   #  이거 짤라다 붙임  요렇게 프린터 되야 하는딩...
    """
    System battery information.
    An object of this type is returned by :py:attr:`Vehicle.battery`.
    :param voltage: Battery voltage in millivolts.
    :param current: Battery current, in 10 * milliamperes. ``None`` if the autopilot does not support current measurement.
    :param level: Remaining battery energy. ``None`` if the autopilot cannot estimate the remaining battery.
    """

    def __init__(self, voltage, current, level):
       
        self.voltage = voltage / 1000.0
        if current == -1:
            self.current = None
        else:
            self.current = current / 100.0
        if level == -1:
            self.level = None
        else:
            self.level = level

    def __str__(self):
        return "Battery:voltage={},current={},level={}".format(self.voltage, self.current, self.level)

#State Machine Itself. Subscribes to various topics and listens for an event update at which point it fires off the next method in the current state. Also runs the run method of the current state at 100ms intervals
class StateMachine():
    def __init__(self):
        rospy.init_node('state_machine')
        self.rate = rospy.Rate(10) #10 Hz cycle
        rospy.Subscriber("tag_found", Bool, self.update_tag_found)    
        rospy.Subscriber("operation_mode", String, self.update_operation_mode)
        rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, self.update_batter) # 베터리 상테 메시지를 밭아옴
        rospy.Subscriber("bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.update_flying)  # 비행상테
        rospy.Subscriber("bebop/states/ardron3/PilotingState/AlertStateChanged", Ardrone3PilotingStateAlertStateChanged, self.update_alert)      # 경고상태
        self.rotation_controller = rospy.Publisher("rotation_controller", String, queue_size=1)
        self.location_controller = rospy.Publisher("location_controller", String, queue_size=1)
        self.cmd_vel_topic = rospy.Publisher("cmd_vel_topic", String, queue_size=1)
        self.state = GroundedState(self.rotation_controller,self.location_controller)
        self.tag_found = False

    def run(self):
        while not rospy.is_shutdown():
            self.state.run()
            self.rate.sleep()

    def update_tag_found(self, data):
        if not (self.tag_found == data.data):
            self.tag_found == data.data
            if self.tag_found == True:
                self.state = self.state.next('tag found')
            else:
                self.state = self.state.next('tag lost')

    def update_operation_mode(self, data):
        if data.data == 'manual':
            self.cmd_vel_topic.publish(rospy.get_param("operator_topic"))
            self.state = self.state.next('manual')
        elif data.data == 'follow':
            self.cmd_vel_topic.publish(rospy.get_param("controller_topic"))
            self.state = self.state.next('follow')
        else:
            rospy.logwarn("Unknown mode " + data.data)

    def update_alert(self, data):
        if data.state == data.state_low_battery or data.state == data.state_critical_battery:
            self.state = self.state.next('low battery')
        else:
            rospy.logwarn("Other warning: " + str(data.state))

    def update_flying(self, data):
        if data.state == data.state_landed:
            self.state_landed = self.state.next("grounded")
        elif data.state == data.state_hovering or data.state == data.state_flying:
            self.state = self.state.next("flying")
        elif data.state == data.state_emergency_landing:
            rospy.logwarn("Emergency Landing State Entered")
        else:
            rospy.loginfo(data)
    
    def update_batter(self, data):
        print(data)
        


if __name__ == '__main__':
    sm = StateMachine()
    #bt = Battery()
    sm.run()
    #bt.run()
    
    
    
    
    
    
    
    
