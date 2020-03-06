#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from dsr_msgs.msg import *
from dsr_msgs.srv import *
import threading
ROBOT_ID = "dsr01"
ROBOT_MODEL = "a0509"

import os, sys

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"/home/hajun/catkin_ws/src/doosan-robot/common/imp")) )
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

from Parent_Robot import Parent_Robot

import socket


class Robot_Slave_a0509(Parent_Robot):
	def __init__(self):
		self.robot_config['name'] = 'a0509'
		self.init_Ros()
		self.gripper_power_on()

	def init_Ros(self):
		rospy.init_node('single_robot_basic_py')
		rospy.on_shutdown(self.robot_stop)

		self.pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=10)
		self.set_tcp = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tcp/set_current_tcp', SetCurrentTcp)
		self.add_tcp = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tcp/config_create_tcp', ConfigCreateTcp)
		self.move_line = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/motion/move_line', MoveLine)
		self.move_joint = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/motion/move_joint', MoveJoint)

		self.move_circle = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_circle", MoveCircle)
		self.move_spline_joint = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_spline_joint",
													MoveSplineJoint)
		self.move_spline_pos = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_spline_task",
												  MoveSplineTask)
		self.move_blend = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_blending", MoveBlending)

		self.digital_out = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/io/set_digital_output',
											  SetCtrlBoxDigitalOutput)
		self.analog_input = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/io/get_analog_input",
											   GetCtrlBoxAnalogInput)
		self.set_mode_analog_input = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/io/set_analog_input_type",
														SetCtrlBoxAnalogInputType)

		self.set_mode_analog_input(0, 1)

		self.set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/system/set_robot_mode', SetRobotMode)
		#self.set_robot_mode(ROBOT_MODE_MANUAL)
		print('해당 로봇은 ROBOT_MODE_AUTONOMOUS로 작동됩니다.')
		self.set_robot_mode(ROBOT_MODE_AUTONOMOUS)

		self.move_jog = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/motion/jog', Jog)

		self.start_recv_state()
		#로봇 입력 대기
		while (self.robot_state['alive'] == False):
			time.sleep(1)
		print('\tRos Initalization Complete')
		return True

	def thread_subscriber(self):
		rospy.Subscriber('/' + ROBOT_ID + ROBOT_MODEL + '/state', RobotState, self.get_robot_state)
		rospy.spin()

	def get_robot_state(self, msg):
		self.robot_state['joint'] = [msg.current_posj[0], msg.current_posj[1], msg.current_posj[2], msg.current_posj[3],
								  msg.current_posj[4], msg.current_posj[5]]
		self.robot_state['pos']= [msg.current_posx[0], msg.current_posx[1], msg.current_posx[2], msg.current_posx[3],
								msg.current_posx[4], msg.current_posx[5]]
		self.robot_state['speed'] = abs(msg.current_velx[0]) + abs(msg.current_velx[1]) + abs(msg.current_velx[2]) + abs(
			msg.current_velx[3]) + abs(msg.current_velx[4]) + abs(msg.current_velx[5])
		# print(self.speed)
		if (self.robot_state['pos'] < 180):
			self.pub_stop.publish(stop_mode=1)
		self.robot_state['alive'] = True

#상속 구현.
	def move_joint_robot(self, posj, vel=100, acc=100, time=0, moveType='Joint', syncType=0):
		mode = 0;
		ref = 0;
		radius = 0.0;
		blendType = 0;
		sol = 0
		syncType = 1 - syncType

		print('\t[command]Move joint %s' % str(posj))
		# config
		vel = vel * self.robot_config['time_rate']
		acc = acc * self.robot_config['time_rate']
		if (time != 0):
			time = time / self.robot_config['time_rate']

		self.move_joint(posj, vel, acc, time, radius, mode, blendType, syncType)

	def move_pos_robot(self, posx, time=0.0, vel=[10, 10], acc=[50, 50], moveType='Line', syncType=0):
		mode = 0;
		ref = 0;
		radius = 0.0;
		blendType = 0;
		sol = 0
		print('\t[command]Move 2 %s' % str(posx))
		syncType = 1 - syncType

		# config
		vel = [vel[0] * self.robot_config['time_rate'], vel[1] * self.robot_config['time_rate']]
		acc = [acc[0] * self.robot_config['time_rate'], acc[1] * self.robot_config['time_rate']]
		time = time / self.robot_config['time_rate']

		self.move_line(posx, vel, acc, time, radius, ref, mode, blendType, syncType)

	def ros_listToFloat64MultiArray(self, list_src):
		_res = []
		for i in list_src:
			item = Float64MultiArray()
			item.data = i
			_res.append(item)
		return _res

	def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		syncType = 1 - syncType
		joint_list = self.ros_listToFloat64MultiArray(joint_list)
		if (time == None):
			vel = vel * self.robot_config['time_rate']
			acc = acc * self.robot_config['time_rate']
		else:
			time = time / self.robot_config['time_rate']
		self.move_spline_joint(joint_list, len(joint_list), vel, acc, time, mod, syncType)

	def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		syncType = 1 - syncType
		pos_list = self.ros_listToFloat64MultiArray(pos_list)
		if (time == None):
			vel = vel * self.robot_config['time_rate']
			acc = acc * self.robot_config['time_rate']
		else:
			time = time / self.robot_config['time_rate']
		self.move_spline_pos(pos_list, len(pos_list), [vel, vel], [acc, acc], time, ref, mod, vel_opt, syncType)

	def move_pos_circle(self, waypos, destpos, vel=[50, 50], acc=[30, 30], syncType=0):
		c1 = [waypos[0], waypos[1], waypos[2], waypos[3], waypos[4], waypos[5]]  # [749,-22,562,13,151,-175]
		c2 = [int(destpos[0]), int(destpos[1]), int(destpos[2]), int(destpos[3]), int(destpos[4]),
			  int(destpos[5])]  # [937,-6.43,461.58,32.85,173.22,-161.97]
		CirclePos = self.ros_listToFloat64MultiArray([c1, c2])

		l_time = 0.0;
		radius = 50.0;
		ref = 0;
		mod = 0
		angle = [0, 0]
		ra = 0
		syncType = 1 - syncType

		# config
		vel = [vel[0] * self.robot_config['time_rate'], vel[1] * self.robot_config['time_rate']]
		acc = [acc[0] * self.robot_config['time_rate'], acc[1] * self.robot_config['time_rate']]
		l_time = l_time / self.robot_config['time_rate']

		self.move_circle(CirclePos, vel, acc, l_time, mod, ref, angle[0], angle[1], radius, ra, syncType)

	def move_jog_robot(self, JOG_AXIS, REF, SPEED):
		# 두산 로봇 기준 메뉴얼 모드에서만 실행이 되므로, 별도로 구현해야함.
		self.move_jog(JOG_AXIS, REF, SPEED)

	def start_recv_state(self):
		self.Thread_subscriber = threading.Thread(target=self.thread_subscriber)
		self.Thread_subscriber.daemon = True
		self.Thread_subscriber.start()

	def robot_stop(self, mode='force'):
		print ("shutdown time!")
		self.grip_release()
		self.pub_stop.publish(stop_mode=3)
		return 0

	# 아래의 코드는 별도로 그리퍼 모듈이 초기화 되지 않으면 호출되는 기본 그리퍼 함수입니다.
	def gripper_power_on(self) :
		self.digital_out(1, 1)
		self.digital_out(4, 1)

	def gripper_power_off(self) :
		self.digital_out(1, 0)
		self.digital_out(4, 0)

	def grip(self):
		self.digital_out(15, 1)
		print('\t[Gripper] Grip')

	def grip_release(self):
		# self.digital_out(15, 0)
		# self.digital_out(8, 1)
		# time.sleep(1)
		# self.digital_out(8, 0)
		print('\t[Gripper] Release')

	def get_grip_success(self):
		#아래와 같이 구현한 이유는, 가끔 값이 리턴이 안되기 때문이다.
		try:
			analog_value = self.analog_input(0)
		except:
			# print("\tanalog Error. its not my falut!")
			return self.get_grip_success()
		# print('suction sensor %s' %analog_value.value)
		if (analog_value.value > 0.5):
			print('\t\t[Info] Gripped')
			return True
		else:
			return False

# class MyTCPHandler(SocketServer.BaseRequestHandler):
# 	m_robot_slave = Robot_Slave_a0509()
# 	def handle(self):
# 		print('클라이언트 접속:{0}'.format(self.client_address[0]))
# 		sock = self.request
#
# 		rbuff = sock.recv(1024)  # 데이터를 수신하고 그 결과를 rbuff에 담습니다. rbuff는 bytes 형식입니다.
# 		print('수신 : {0}'.format(rbuff))
#
# 		split_str = rbuff.split(', ')
# 		print(split_str)
# 		if split_str[0].split('[')[1].split("'")[1] == 'a0509':
# 			print('robot type is a0509')
# 			if split_str[1].split("'")[1] == 'Joint':
# 				print('command tpye is Joint')
#                 cmd_joint = [float(split_str[2]), float(split_str[3]), float(split_str[4]), float(split_str[5]), float(split_str[6]), float(split_str[7])]
#                 print cmd_joint
#                 self.m_robot_slave.move_joint_robot(cmd_joint, vel=50, acc=10, syncType=1)



		# if split_str[0].split('[')[1]

		# self.m_robot_slave.move_joint_robot([185.36, 3.16, -125.24, 180.27, 60.8, 0.48], vel=50, acc=10, syncType=1)


if __name__ == '__main__':
	m_robot_slave = Robot_Slave_a0509()

	HOST = '127.0.0.1'
	PORT = 4561

	# 소켓 객체를 생성합니다.
	# 주소 체계(address family)로 IPv4, 소켓 타입으로 TCP 사용합니다.
	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	# 포트 사용중이라 연결할 수 없다는
	# WinError 10048 에러 해결를 위해 필요합니다.
	server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	# bind 함수는 소켓을 특정 네트워크 인터페이스와 포트 번호에 연결하는데 사용됩니다.
	# HOST는 hostname, ip address, 빈 문자열 ""이 될 수 있습니다.
	# 빈 문자열이면 모든 네트워크 인터페이스로부터의 접속을 허용합니다.
	# PORT는 1-65535 사이의 숫자를 사용할 수 있습니다.
	server_socket.bind((HOST, PORT))

	# 서버가 클라이언트의 접속을 허용하도록 합니다.
	server_socket.listen(0)

	# accept 함수에서 대기하다가 클라이언트가 접속하면 새로운 소켓을 리턴합니다.
	client_socket, addr = server_socket.accept()

	# 접속한 클라이언트의 주소입니다.
	print('Connected by', addr)

	# 무한루프를 돌면서
	while True:

		# 클라이언트가 보낸 메시지를 수신하기 위해 대기합니다.
		data = client_socket.recv(1024)

		# 빈 문자열을 수신하면 루프를 중지합니다.
		if not data:
			print 'Finished comm'
			break
		else:
			# 수신받은 문자열을 출력합니다.
			print('Received from', addr, data.decode())
			split_str = data.split(', ')
			print(split_str)

			robot_type = split_str[0].split('[')[1].split("'")[1]
			print('robot type is {0}'.format(robot_type))
			if robot_type == 'a0509':
				cmd_type = split_str[1].split("'")[1]
				print('command type is {0}'.format(cmd_type))
				if cmd_type == 'Joint':
					cmd_joint = [float(split_str[2]), float(split_str[3]), float(split_str[4]), float(split_str[5]), float(split_str[6]), float(split_str[7])]
					vel = int(split_str[8])
					acc = int(split_str[9])
					time = int(split_str[10])
					sync = int(split_str[11].split(']')[0])
					print cmd_joint, vel, acc, time, sync
					m_robot_slave.move_joint_robot(cmd_joint, vel=vel, acc=acc, time=2, syncType=sync)

				elif cmd_type == 'Line':
					cmd_pose = [float(split_str[2]), float(split_str[3]), float(split_str[4]), float(split_str[5]), float(split_str[6]), float(split_str[7])]
					vel_pos = int(split_str[8])
					vel_ang = int(split_str[9])
					vel = [vel_pos, vel_ang]
					acc_pos = int(split_str[10])
					acc_ang = int(split_str[11])
					acc = [acc_pos, acc_ang]
					time = int(split_str[12])
					sync = int(split_str[13].split(']')[0])
					print cmd_pose, vel, acc, time, sync
					m_robot_slave.move_pos_robot(cmd_pose, vel=vel, acc=acc, syncType=sync)

				elif cmd_type == 'JointList':
					cmd_joint1 = [float(split_str[2]), float(split_str[3]), float(split_str[4]), float(split_str[5]),
								 float(split_str[6]), float(split_str[7])]
					cmd_joint2 = [float(split_str[8]), float(split_str[9]), float(split_str[10]), float(split_str[11]),
								 float(split_str[12]), float(split_str[13])]
					vel = int(split_str[14])
					acc = int(split_str[15])
					time = int(split_str[16])
					sync = int(split_str[17].split(']')[0])
					cmd_list = [cmd_joint1, cmd_joint2]
					print cmd_list, vel, acc, time, sync
					m_robot_slave.move_joint_queue(cmd_list, vel=vel, acc=acc, time=time, syncType=sync)

				elif cmd_type == 'LineList':
					cmd_pose1 = [float(split_str[2]), float(split_str[3]), float(split_str[4]), float(split_str[5]), float(split_str[6]), float(split_str[7])]
					cmd_pose2 = [float(split_str[8]), float(split_str[9]), float(split_str[10]), float(split_str[11]), float(split_str[12]), float(split_str[13])]
					vel = int(split_str[14])
					acc = int(split_str[15])
					time = int(split_str[16])
					mod = int(split_str[17])
					ref = int(split_str[18])
					vel_opt = int(split_str[19])
					sync = int(split_str[20].split(']')[0])
					cmd_list = [cmd_pose1, cmd_pose2]
					print cmd_list, vel, acc, time, sync
					m_robot_slave.move_pos_queue(cmd_list, vel=vel, acc=acc, time=time, syncType=sync)



		# 받은 문자열을 다시 클라이언트로 전송해줍니다.(에코)
		# client_socket.sendall(data)

	# 소켓을 닫습니다.
	client_socket.close()
	server_socket.close()
