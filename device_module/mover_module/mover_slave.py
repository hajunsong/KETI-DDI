#!/usr/bin/env python
# -*- coding: utf-8 -*-
#이게 있어야 한글 써짐.

import u12
import threading, socket, time

class mover_slave():
	def __init__(self):
		self.conveyor = u12.U12()

	def init(self, HOST='127.0.0.1', PORT=8888):
		print('나는 컨베이어 벨트다')

		self.HOST = HOST
		self.PORT = PORT

		self.t1 = threading.Thread(target=self.socket_setting)
		self.t1.daemon = True
		self.t1.start()

	def run(self):
		print('컨베이어 벨트 작동한다.')
		self.conveyor.eAnalogOut(analogOut0=0.0, analogOut1=2.0)

	def stop(self):
		print('컨베이어 벨트 꺼진다.')
		self.conveyor.eAnalogOut(analogOut0=0.0, analogOut1=0.0)

	def socket_setting(self):
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.server_socket.bind((self.HOST, self.PORT))
		self.server_socket.listen(0)

		print '[MS]wait connect client...'
		self.client_socket, self.addr = self.server_socket.accept()
		print('[MS]Connected by', self.addr)
		self.connected = True

		while True:
			time.sleep(0.1)
			data = self.client_socket.recv(1024)
			print('[MS]Received from', self.addr, data)

			if not data:
				print '[MS]Finished comm'
				self.client_socket.close()
				self.server_socket.close()
				print '[MS]Restart Server'

				self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
				self.server_socket.bind((self.HOST, self.PORT))
				self.server_socket.listen(0)

				print '[MS]wait connect client...'
				self.client_socket, self.addr = self.server_socket.accept()
				print('[MS]Connected by', self.addr)
			else:
				if data == '0':
					self.stop()
				elif data == '1':
					self.run()