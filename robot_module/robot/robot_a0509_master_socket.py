#!/usr/bin/env python
# -*- coding: utf-8 -*-
#이 파이선 코드는
import socket

class Robot_Master_a0509():
	def __init__(self, HOST='127.0.0.1', PORT=8888):
		self.robot_name = 'a0509'

		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # SOCK_STREAM은 TCP socket을 뜻함
		self.sock.bind((HOST, 0))

		connected = False

		while ~connected:
			try:
				self.sock.connect((HOST, PORT))  # 서버에 연결 요청
				connected = True
				break
			except:
				connected = False

		print 'connected server'


	# 명령어 코드
	# 각 로봇에 맞춰 적절한 연산을 해주면 된다.
	# 적절한 연산의 예: M로봇을 A로봇의 좌표계로 변환후 작동.
	def move_joint_robot(self,posj, vel=100, acc = 100, time = 0, moveType = 'Joint', syncType = 0):

		# 서버로 송신
		send_message = []
		send_message.append(self.robot_name)
		send_message.append(moveType)
		send_message.append(posj[0])
		send_message.append(posj[1])
		send_message.append(posj[2])
		send_message.append(posj[3])
		send_message.append(posj[4])
		send_message.append(posj[5])
		send_message.append(vel)
		send_message.append(acc)
		send_message.append(time)
		send_message.append(syncType)

		try:
			sbuff = bytes(send_message)
			self.sock.send(sbuff)  # 메시지 송신
			print('송신:{0}'.format(send_message))
			self.sock.flush()
		except:
			pass
		# self.m_Robot_Slave.move_joint_robot(posj, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)

	def move_pos_robot(self, posx, time = 0.0, vel=[10,10], acc= [50,50], moveType = 'Line', syncType = 0):
		# self.m_Robot_Slave.move_pos_robot(posx, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)

		# 서버로 송신
		send_message = []
		send_message.append(self.robot_name)
		send_message.append(moveType)
		send_message.append(posx[0])
		send_message.append(posx[1])
		send_message.append(posx[2])
		send_message.append(posx[3])
		send_message.append(posx[4])
		send_message.append(posx[5])
		send_message.append(vel[0])
		send_message.append(vel[1])
		send_message.append(acc[0])
		send_message.append(acc[1])
		send_message.append(time)
		send_message.append(syncType)

		try:
			sbuff = bytes(send_message)
			self.sock.send(sbuff)  # 메시지 송신
			print('송신:{0}'.format(send_message))
		except:
			pass

	def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		# self.m_Robot_Slave.move_pos_queue(pos_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)
		# 서버로 송신
		send_message = []
		send_message.append(self.robot_name)
		send_message.append('LineList')
		for i in range(0, len(pos_list)):
			send_message.append(pos_list[i][0])
			send_message.append(pos_list[i][1])
			send_message.append(pos_list[i][2])
			send_message.append(pos_list[i][3])
			send_message.append(pos_list[i][4])
			send_message.append(pos_list[i][5])
		send_message.append(vel)
		send_message.append(acc)
		send_message.append(time)
		send_message.append(mod)
		send_message.append(ref)
		send_message.append(vel_opt)
		send_message.append(syncType)

		try:
			sbuff = bytes(send_message)
			self.sock.send(sbuff)  # 메시지 송신
			print('송신:{0}'.format(send_message))
		except:
			pass

	def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		# self.m_Robot_Slave.move_joint_queue(joint_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)
		# 서버로 송신
		send_message = []
		send_message.append(self.robot_name)
		send_message.append('JointList')
		for i in range(0, len(joint_list)):
			send_message.append(joint_list[i][0])
			send_message.append(joint_list[i][1])
			send_message.append(joint_list[i][2])
			send_message.append(joint_list[i][3])
			send_message.append(joint_list[i][4])
			send_message.append(joint_list[i][5])
		send_message.append(vel)
		send_message.append(acc)
		send_message.append(time)
		send_message.append(mod)
		send_message.append(ref)
		send_message.append(vel_opt)
		send_message.append(syncType)

		try:
			sbuff = bytes(send_message)
			self.sock.send(sbuff)  # 메시지 송신
			print('송신:{0}'.format(send_message))
		except:
			pass

	def move_pos_circle(self, waypos, destpos, vel=[50, 50], acc=[30, 30], syncType=0):
		# self.m_Robot_Slave.move_pos_circle(waypos, destpos, vel=vel, acc=acc, syncType=syncType)
		pass

	def move_jog_robot(self, JOG_AXIS, REF, SPEED):
		# self.m_Robot_Slave.move_jog_robot(JOG_AXIS, REF, SPEED)
		pass

	def robot_stop(self, mode = 'force'):
		# self.m_Robot_Slave.robot_stop(mode = mode)

		# 서버로 송신
		send_message = []
		send_message.append(self.robot_name)
		send_message.append('stop')
		send_message.append(mode)

		try:
			sbuff = bytes(send_message)
			self.sock.send(sbuff)  # 메시지 송신
			print('송신:{0}'.format(send_message))
		except:
			pass
