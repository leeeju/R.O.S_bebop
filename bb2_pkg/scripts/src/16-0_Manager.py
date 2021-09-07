#!/usr/bin/python
#-*- coding: utf-8 -*-
'''
비행 및 탐지 기능을 관리하는 코드이다.
'''
import rospy, subprocess

if __name__ == '__main__':
	rospy.init_node('Fire_drone_manager')
	while True:
		#파라미터의 조건이 되면 코드를 실행한다.
		cod1 = rospy.get_param("/Fire_drone_managerl/order")
		if cod1 == 1:
			#이미 순찰 비행 코드가 실행 중인지 확인한다. 만일 실행 중이지 않다면, 순찰 비행 코들르 실행한다.
			#아래는 로스런치로 실행한 비행 노드가 있는지 확인하고 없다면 실행하는 코드이다.
			proc0 = subprocess.Popen(['rosnode', 'list', '/fly_to_targetl'], stdout=subprocess.PIPE)
			for line in proc0.stdout:
				pass
			if len(line.rstrip()) == 0:
				#순찰 비행 코드를 실행한다.
				subprocess.call(["rosrun", "bb2_pkg", "16-1_FlyTarget.py"])
			#실행을 완료했다면 파라미터를 초기화한다.
			rospy.get_param("/Fire_drone_managerl/order", 0)
		else:
			pass
		