#!/usr/bin/env python
# -*- coding: utf-8 -*-
#이게 있어야 한글 써짐.
import sys
import os
import time, threading, socket
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))

from device_instance import device_instance
from mover_slave import mover_slave

class mover_master(device_instance):
	#상속 받아 구현하는 함수.
	device_slave = mover_slave()
	def init_device(self, HOST='127.0.0.1', PORT=8888):
		self.device_slave.init(HOST, PORT)
		self.HOST = HOST
		self.PORT = PORT
		self.connected = False

		self.t1 = threading.Thread(target=self.socket_setting)
		self.t1.daemon = True
		self.t1.start()
		return

	def socket_setting(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # SOCK_STREAM은 TCP socket을 뜻함
		self.sock.bind((self.HOST, 0))
		while ~self.connected:
			try:
				self.sock.connect((self.HOST, self.PORT))  # 서버에 연결 요청
				self.connected = True
				break
			except:
				self.connected = False
		print '[MM]connected server'
		self.send_message = '0'

		# self.send_message = '1'
		# sbuff = bytes(self.send_message)
		# self.sock.send(sbuff)  # 메시지 송신
		# print('[MM]송신:{0}'.format(self.send_message))

	def run(self, args=0):
		# self.device_slave.run()
		self.send_message = '1'
		sbuff = bytes(self.send_message)
		self.sock.send(sbuff)  # 메시지 송신
		print('[MM]송신:{0}'.format(self.send_message))
		return

	def stop(self, args=0):
		# self.device_slave.stop()
		self.send_message = '0'
		sbuff = bytes(self.send_message)
		self.sock.send(sbuff)  # 메시지 송신
		print('[MM]송신:{0}'.format(self.send_message))
		return


if __name__ == '__main__':
	print('컨베이어 벨트 코드 테스트')
	m_mover_master = mover_master()
	m_mover_master.init_device(HOST='127.0.0.1', PORT=8888)
	time.sleep(1)
	m_mover_master.run()
	time.sleep(1)
	m_mover_master.stop()