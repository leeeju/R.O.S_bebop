#!/usr/bin/env python

import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, Temperature, Range

drone_state = State()
drone_battery = BatteryState()
rangefinder = Range()

def drone_validate():

    imu_temperature = 25
    #imu_temperature e  a temperatura interna da pixhawk
    max_temperature = 60

    rate = rospy.Rate(20)
    rate.sleep()
    def state_callback(state_data):
        global drone_state
        drone_state = state_data

    def battery_callback(battery_data):
        global drone_battery
        drone_battery.voltage = battery_data.voltage
        drone_battery.current = battery_data.current
        drone_battery.percentage = battery_data.percentage

    def temperature_callback(temp_data):
        global imu_temperature
        imu_temperature = temp_data.temperature

    def range_callback(range_data):
        global rangefinder
        rangefinder.range = range_data.range

    #arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    state_subscriber = rospy.Subscriber('/mavros/state', State, state_callback)
    temperature_subscriber = rospy.Subscriber('/mavros/imu/temperature', Temperature, temperature_callback)
    range_subscriber = rospy.Subscriber('/mavros/distance_sensor/hrlv_ez4_pub', Range, range_callback)
    battery_subscriber = rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', BatteryState, battery_callback)
    
    rospy.exceptions.ROSInitException


    rate.sleep()

    while not rospy.is_shutdown():

	    if imu_temperature > max_temperature:
		    rospy.logwarn('DRONE IS OVERHEATED')
		    return 'overheat'

	    if drone_battery.percentage <= 0.05 and drone_battery.voltage != 0:
    		rospy.logwarn('LOW BATTERY, '+ str(drone_battery.percentage*100) + '% , ' + str(drone_battery.voltage) + 'V')
    		return 'low_battery'

	    if rangefinder.range <= 0.8:
    		rospy.logwarn('OBSTACLE AHEAD AT ' + str(rangefinder.range) + 'm')
    		return 'obstacle'
            rate.sleep()


    while not drone_state.armed:
        print('[ WARNING ] DRONE IS DISARMED')
        arm(True)
        print('[ WARNING ] ARMING DRONE')
        if drone.armed:
            return 'validated'
        rate.sleep()

if __name__ == '__main__':
    drone_validate()
