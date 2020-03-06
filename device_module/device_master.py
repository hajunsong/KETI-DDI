#!/usr/bin/env python
# -*- coding: utf-8 -*-
#이게 있어야 한글 써짐.

from device_instance import device_instance

import sys
import os, time
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'flipper_module')))
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'mover_module')))

#읽어 보시죠.
#각 모듈의 마스터입니다.
#각 모듈의 마스터(로봇, 카메라, 디바이스)는 싱글톤 타입으로 관리되는 것을 원칙으로 합니다.
#모듈 마스터는 메인 마스터에서 내려오는 명령어를 각 하위 마스터들에게 분배합니다.
#각 하위 마스터들은 프로토타입 모델에 기반하여 초기화 됩니다.
#따라서 새로운 형태의 하위 마스터들이 추가될때마다 초기화 코드를 추가해야 합니다.
#디바이스 마스터는 아직 제대로 작동하는 모듈이 없습니다.
#따라서, 구현을 해나가면서 필요한 코드를 추가해 가시면 됩니다.

#각 모듈 마스터는 단독 실행이 가능하게 구현해 두길 추천합니다. (밑에 main에 예시 코드를 넣어둡니다.)

#각 하위 마스터를 구현하실때 아래의 방법을 쫒아하시오.
#1. 만들고자 하는 하위 마스터의 마스터 파일과, 슬레이브 파일을 만드십시오. ex. aa_master.py, aa_slave.py
#2. 만든 하위 마스터의 슬레이브 파일에서 코드를 작성하십시오. 하드웨어 초기화, 카메라 초기화 등등 같은 최저 수준의 구현을 하시면 됩니다.
#3. 만든 하위 마스터는 해당 모듈의 instance클래스를 상속하게 하십시오. 예를 들어 디바이스 마스터는 device_instance를 상속하면 됩니다.
#4. 만든 하위 마스터에서 상속받은 함수들을 2에서 정의한 함수들을 조합 처리 하여, 초기화 하십시오. 초기화해야할 코드들은 각 모듈의 instance 클래스를 참고하시면 됩니다.
#5. 만든 하위 마스터들을 해당하는 모듈 마스터에서 초기화 할 수 있게 하십시오. 초기 선언 부, 예시 코드를 참고하시면 됩니다.

#각 모듈마스터에서 함수를 추가하실때에는..
#1. 각 모듈 마스터의 instance클래스에 사용하고자 하는 함수의 기본형을 작성하십시오.
#2. 모듈 마스터에서 메인 마스터에서 호출 할 수 있게 정의 하십시오. 다른 함수들을 참고하시면 됩니다.


class device_master(object):
	device_module = []
	#디바이스마스터는 싱글톤으로 관리됩니다.
	def __new__(self):
		if not hasattr(self, 'instance'):
			self.instance = super(device_master, self).__new__(self)
		return self.instance

#초기화 코드들.
	def init_device(self, device_dict = []):
		#싱글톤 체크.
		if(len(self.device_module)>0):
			print("[Warning] 디바이스는 싱글톤으로 관리되며, 한번만 초기화 됩니다.")
			return

		for i in device_dict:
			if(str(type(i))!="<type 'dict'>"):
				temp_dict = {}
				temp_dict['NAME'] = i
				temp_dict['TYPE'] = i
				i = temp_dict
			if(i['TYPE']=='flipper'):
				ret_obj = self.init_flipper(device_config = i)
			elif(i['TYPE']=='mover'):
				ret_obj = self.init_mover(device_config=i)
			###########################
			#새로운 타입의 디바이스가 추가될때마다 여기에다가 추가하세요.
			##############################
			else:
				ret_obj = device_instance(p_dict = i)

			self.device_module.append(ret_obj)
		return

	def init_flipper(self, device_config = ''):
		ret_obj = []
		if(device_config['NAME']== 'flipper'):
			print('당신은 최초의 뒤집개 클래스를 호출하였습니다.')
			from flipper_master import flipper_master
			ret_obj = flipper_master(p_dict = device_config)
			ret_obj.init_device(HOST='127.0.0.1', PORT=9999)
			time.sleep(0.5)
		return ret_obj

	def init_mover(self, device_config = ''):
		ret_obj = []
		if(device_config['NAME']== 'mover'):
			print('당신은 최초의 컨베이어 시스템 클래스를 호출하였습니다.')
			from mover_master import mover_master
			ret_obj = mover_master(p_dict = device_config)
			ret_obj.init_device(HOST='127.0.0.1', PORT=8888)
		return ret_obj

#파라미터는 예시이다, 추후에 필요하면 추가하자.
	def command_run(self, device_type = '', args = 0):
		for i in self.device_module:
			if (i.m_type == device_type):
				return i.run(args = args)
		print('%s 타입의 디바이스는 검색이 되지 않습니다.'%device_type)
		return False

	def command_stop(self, device_type = '', args = 0):
		for i in self.device_module:
			if (i.m_type == device_type):
				return i.stop(args = args)
		print('%s 타입의 디바이스는 검색이 되지 않습니다.'%device_type)
		return False

	def command_flip_flippers(self, rotation_z = 180):
		return self.command_run(device_type='flipper', args = rotation_z)

	def command_move_mover(self, size = 100):
		return self.command_run(device_type='mover', args=size)

from pynput.keyboard import Listener, Key
def on_press(key):
    global key_value
    try:
        if key == Key.esc:
            key_value = 'q'
        else:
            key_value = key.char
    except Exception as e:
        print e.message
    return False

if __name__ == '__main__':
    global key_value
    print('디바이스 마스터 코드 테스트')
    cc = {}
    cc['NAME'] = 'mover'
    cc['TYPE'] = 'mover'

    m_device_master = device_master()
    m_device_master.init_device(device_dict = ['flipper', cc])
    time.sleep(1)

    key_value = 0
    # aa.command_flip_flippers()
	# time.sleep(15)
	# aa.command_stop(device_type = 'flipper')
	# time.sleep(1)
	# aa.command_move_mover()
	# time.sleep(1)
	# aa.command_stop(device_type='mover')
	# time.sleep(1)
	# aa.init_device(device_dict=['mover', 'flipper'])
	# time.sleep(1)

    while True:
        with Listener(on_press=on_press) as listener:
            listener.join()
        try:
            if key_value == 'a' or key_value == '1':
                m_device_master.command_flip_flippers()
            elif key_value == 'b' or key_value == '2':
                m_device_master.command_move_mover()
                time.sleep(0.4)
                m_device_master.command_stop(device_type='mover')
            elif key_value == 'q' or key_value == 'c':
                break

            key_value = 0
        except Exception as e:
            print e.message

    print 'Good bye !!!'