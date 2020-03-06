#!/usr/bin/env python
# -*- coding: utf-8 -*-
#이게 있어야 한글 써짐.
import sys
import os
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
from device_instance import device_instance
from flipper_slave import flipper_slave
import socket, threading, time

#읽어 보시죠.
#각각 디바이스들의 마스터입니다.
#각 마스터는, 상위단 마스터의 명령을 하위 슬레이브와 맞춰주는 역활을 합니다.
#예를 들어, 상위단에서 move라는 명령어가 호출이 되면, move명령이 실행되도록 '보장'되야 합니다.
#다른 예로, 상위 마스터와, 하위 슬레이브의 좌표계가 다른 경우, 좌표계 변경은 각 마스터 들이 해줘야 합니다.
#디바이스 마스터는 아직 제대로 작동하는 모듈이 없습니다.
#각자 구현 중 필요한 부분은 마스터, 슬레이브 구조에 맞춰 구현하세요.
#
#각 디바이스들의 마스터 구현은 아래 과정을 쫒아하세요.
#1. 하위 디바이스의 마스터 클래스를 만듭니다. 그리고 해당 클래스는 'device_instance'클래스를 상속하세요.
#   device_instance는 디바이스 마스터들의 부모 클래스 입니다.
#2. device_instance.py에 있는 함수 프로토 타입들을 초기화 하세요. 해당 함수들을 방금 작성한 클래스로 복사후 구현하면 됩니다.
#3. 각 구현된 함수들은 슬래이브들의 함수를 잘 조합하여 작동 되게 하면 됩니다.
#
#이 코드의 작동예시는 아래에 있습니다.

class flipper_master(device_instance):
	#상속 받아 구현하는 함수.
	device_slave = flipper_slave()
	def init_device(self, HOST='127.0.0.1', PORT=9999):
		self.device_slave.init(HOST, PORT)

		self.connected = False

		self.HOST = HOST
		self.PORT = PORT
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
		print '[FM]connected server'
		self.send_message = '0'

		self.send_message = '1'
		sbuff = bytes(self.send_message)
		self.sock.send(sbuff)  # 메시지 송신
		print('[FM]송신:{0}'.format(self.send_message))

	def run(self, args=0):
		# self.device_slave.run()
		self.send_message = '2'
		sbuff = bytes(self.send_message)
		self.sock.send(sbuff)  # 메시지 송신
		print('[FM]송신:{0}'.format(self.send_message))
		return

	def stop(self, args=0):
		# self.device_slave.stop()
		self.send_message = '0'
		sbuff = bytes(self.send_message)
		self.sock.send(sbuff)  # 메시지 송신
		print('[FM]송신:{0}'.format(self.send_message))


if __name__ == '__main__':
	print('뒤집개 코드 테스트')
	m_flipper_master = flipper_master()
	m_flipper_master.init_device(HOST='127.0.0.1', PORT=9999)
	time.sleep(3)
	m_flipper_master.run()
	time.sleep(12)
	m_flipper_master.stop()