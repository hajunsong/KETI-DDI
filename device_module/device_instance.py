#!/usr/bin/env python
# -*- coding: utf-8 -*-

class device_instance:
	m_name = ''
	m_type = ''
	device_slave = []

	def __init__(self, p_dict = ''):
		if(p_dict == ''):
			p_dict = {}
			p_dict['NAME'] = ''
			p_dict['TYPE'] = ''
		self.m_name = p_dict['NAME']
		self.m_type = p_dict['TYPE']

#아래의 함수를 구현하시오.
	def init_device(self):
		print('이게 호출되었다는건, 해당 디바이스는 정의되지 않은 겁니다.')
		return

	def run(self, args = 0):
		print('이게 호출되었다는건, 해당 디바이스는 정의되지 않은 겁니다.')
		return

	def stop(self, args = 0):
		print('이게 호출되었다는건, 해당 디바이스는 정의되지 않은 겁니다.')
		return