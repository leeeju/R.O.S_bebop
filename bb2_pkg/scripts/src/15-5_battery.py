#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as Battery # 비밥 드론의 메세지 토픽을 받아옴

def callback(data):
	rospy.loginfo("현재 남은 베터리 잔량 : %s%%",data.percent)
	if data.percent < 15:
		rate = rospy.Rate(10)
		rospy.loginfo("베터리 잔량이 15% 이하 입니다. 착륙 준비 하세요.")    #info 형식의 메시지 출력 // ex) [INFO] [1627887334.009190]: 베터리 잔량이 15% 이하 입니다. 
		rospy.loginfo(" ** 베터리 잔량 경고 ** ")

def main():
	rospy.init_node('Battery_Level')
	rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged", Battery, callback) # Subscriber 한다 비밥이 던져 주는 베터리 토픽을
	rospy.spin()

if __name__ == '__main__':

	#목적지의 위경도를 입력 받는다.
	target_lad1 = float(input("input target latitude: "))
	target_lod1 = float(input("input target longitude: "))

	#입력받은 위경도를 파라미터 서버에 저장한다.
	rospy.set_param("/lati", target_lad1)
	rospy.set_param("/long", target_lod1)

	main()