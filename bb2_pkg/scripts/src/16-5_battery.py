#!/usr/bin/python
#-*- coding: utf-8 -*-
'''
bebop 드론의 베터리가 10% 가되면 home 으로 돌아옴
'''
import rospy, subprocess
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as Battery # 비밥 드론의 메세지 토픽을 받아옴

def callback(data):
	global batery_percent
	batery_percent = int(data.percent)

if __name__ == '__main__':

	rospy.init_node('Battery_Level')
	rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged", Battery, callback)

	#목적지의 위경도를 입력 받는다.
	target_lad1 = float(input("input target latitude: "))
	target_lod1 = float(input("input target longitude: "))

	#입력받은 위경도를 파라미터 서버에 저장한다.
	rospy.set_param("/lati", target_lad1)
	rospy.set_param("/long", target_lod1)

	#반복문 안에서 조건문을 한번 실행하고 다시 실행하지 않도록 하기 위한 인덱스 변수를 설정한다.
	i = 0
	
	while True:

		rospy.sleep(10)
		rospy.loginfo("배터리 잔량: %d%% ", batery_percent)

		if (i == 0) and (batery_percent < 15):
			rate = rospy.Rate(10)
			rospy.loginfo("베터리 잔량이 15% 이하 입니다. 되돌아 갑니다. 착륙 준비 하세요.")    #info 형식의 메시지 출력 // ex) [INFO] [1627887334.009190]: 베터리 잔량이 15% 이하 입니다. 
			#처음 시작점으로 되돌아 오는 비행 코드를 활성화한다.
			rospy.set_param("/FlyBackl/param_of_back", "1")
			#불 인식 코드를 비활성화한다.
			rospy.set_param("/fire_detectorl/param_of_detector", "0")
			#FlyTarget 코드를 종료한다.
			subprocess.call(['rosnode', 'kill', '/fly_to_targetl'])
			#반복문에서 위의 코드를 실행했다면, 다시 반복하지 않도록 인덱스 값에 변화를 준다.
			i = 1

	rospy.spin()
		
