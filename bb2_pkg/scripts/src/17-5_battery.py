#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy, subprocess, rosnode
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as Battery # 비밥 드론의 메세지 토픽을 받아옴

def callback(data):
	global batery_percent
	batery_percent = 100
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

	#비밥 드론이 베터리 메시지를 보낼 때까지 일정 시간동안 기다린다.
	rospy.sleep(120)

	#반복문 안에서 조건문을 한번 실행하고 다시 실행하지 않도록 하기 위한 인덱스 변수를 설정한다.
	i = 0

	while not rospy.is_shutdown():
		rospy.loginfo("배터리 잔량: %d%% ", batery_percent)
		rospy.sleep(10)
		if (i == 0) and (batery_percent < 15):
			#베터리의 잔량을 터미널에서 출력한다.
			rospy.loginfo("베터리 잔량이 15% 이하 입니다. 되돌아 갑니다. 착륙 준비 하세요.")    
			#불 인식 코드를 비활성화한다.
			rospy.set_param("/fire_detectorl/param_of_detector", "0")
			#FlyTarget이 실행 중에 있다면 그 노드를 종료한다.
			proc1 = subprocess.Popen(['rosnode', 'list', '/fly_to_targetl'], stdout=subprocess.PIPE)
			for line in proc1.stdout:
				pass
			if len(line.rstrip()) > 0:
				rosnode.kill_nodes(['/fly_to_targetl'])
			#반복문에서 위의 코드를 실행했다면, 다시 반복하지 않도록 인덱스 값에 변화를 준다.
			#처음 시작점으로 되돌아 오는 비행 코드를 활성화한다.
			rospy.set_param("/FlyBackl/param_of_back", "1")
			i = 1
