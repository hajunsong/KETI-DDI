#!/usr/bin/env python
# -*- coding: utf-8 -*-
#이게 있어야 한글 써짐.

#읽어 보시죠.
#각각 디바이스들의 슬레이브입니다.
#각 슬레이브는 작동 코드와 바로 물려있습니다.
#예를 들어 ROS라면 로봇 슬레이브에서 호출 되야하며,
#카메라 라면 하드웨어 접근 후 초기화 하는 과정은 카메라 슬레이브에서 해야합니다.
#슬레이브에 있는 코드들은 단독 작동되게 구현하길 추천하며,
#여기서 구현된 코드는 본 슬레이브의 마스터에서 호출됩니다.

#각 디바이스들의 마스터 구현은 아래 과정을 쫒아하세요.
#1. 해당 디바이스가 작동 및 구현 할 수 있는 함수를 작성합니다.
#2. 본 슬레이브의 마스터가 함수를 사용할 수 있게 구현하시오.

import PythonLibMightyZap
import time, threading
import dynamixel_sdk
import socket


class GripingRobot(threading.Thread):
    g_Trgt_Pos_IRR1_1 = 0
    g_Trgt_Pos_IRR1_2 = 0
    g_Crrt_Pos_IRR1_1 = 0
    g_Crrt_Pos_IRR1_2 = 0
    g_bfr_Pos_IRR1_1 = 0
    g_bfr_Pos_IRR1_2 = 0
    g_flgComm_IRR = False
    g_flgGrip_IRR = False
    g_flgRls_IRR = False
    g_flgForce_IRR = False
    g_flgGripCnt = 0
    g_Crrt_State_IRR1 = 0
    g_flgComm_Move_IRR = False
    g_cnt = 0

    # Control table address
    ADDR_PRO_TORQUE_ENABLE = 562            # Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_POSITION = 596
    ADDR_PRO_PRESENT_POSITION = 611
    g_flgComm_DXL = False
    g_flgMove_DXL = False
    g_flgServo_DXL = False
    g_flgForce_DXL = False
    # Protocol version
    PROTOCOL_VERSION = 2.0                  # See which protocol version is used in the Dynamixel
    # Default setting
    DXL_ID_Axis1 = 1                        # Dynamixel ID : 1
    DXL_ID_Axis2 = 2                        # Dynamixel ID : 1
    BAUDRATE = 57600                        # Dynamixel default baudrate : 57600
    DEVICENAME = '/dev/ttyUSB0'             # Check which port is being used on your controller

    TORQUE_ENABLE = 1                       # Value for enabling the torque
    TORQUE_DISABLE = 0                      # Value for disabling the torque
    DXL_MINIMUM_POSITION_VALUE = 10         # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE = 4000       # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    Axis1_TarPos = int(0)
    Axis2_TarPos = int(0)
    Axis1_CurPos = int(0)
    Axis2_CurPos = int(0)
    DXL_MOVING_STATUS_THRESHOLD = 20        # Dynamixel moving status threshold

    portHandler = dynamixel_sdk.PortHandler(DEVICENAME)
    packetHandler = dynamixel_sdk.PacketHandler(PROTOCOL_VERSION)

    Tick = 10

    def moveGriper(self):
        if self.g_flgComm_DXL:
            self.g_flgComm_Move_IRR = True
            # Grip
            if self.g_Crrt_Pos_IRR1_1 < self.g_Trgt_Pos_IRR1_1 and self.g_Crrt_Pos_IRR1_2 < self.g_Trgt_Pos_IRR1_2:
                self.g_flgForce_IRR = True
                self.g_flgGripCnt = 0
            # Realese
            else:
                self.g_flgForce_IRR = False
                self.g_flgGripCnt = 0

    def PortOpenDXL(self):
        if self.portHandler.openPort():
            print("Succeeded to Open the Port")
            self.g_flgComm_DXL = True
        else:
            print("Failed to Open the Port")
            self.g_flgComm_DXL = False

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE) and self.g_flgComm_DXL:
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            self.g_flgComm_DXL = False

    # Close Port
    def PortCloseDXL(self):
        if self.g_flgComm_DXL:
            self.portHandler.closePort()

    def ErrorPrint(self, comm_result, error, AxisID):
        if comm_result != dynamixel_sdk.COMM_SUCCESS:
            print("AsisID ", AxisID, ": %s" % self.packetHandler.getTxRxResult(comm_result))
            return False
        elif error != 0:
            print("AsisID ", AxisID, ": %s" % self.packetHandler.getRxPacketError(error))
            return False
        else:
            return True

    def DXL_TrqEnable(self):
        if self.g_flgComm_DXL:
            self.g_flgServo_DXL = True
            self.g_flgForce_DXL = True


    def DXL_TrqDisable(self):
        if self.g_flgComm_DXL:
            self.g_flgServo_DXL = True
            self.g_flgForce_DXL = False

    def run(self):
        # init
        # print("Start")
        # self.stime = time.time()
        self.g_flgComm_Move_IRR= False
        # initial
        if self.g_flgComm_DXL:
            self.g_Crrt_Pos_IRR1_1 = PythonLibMightyZap.presentPosition(self.portHandler,0)
            self.g_Crrt_Pos_IRR1_2 = PythonLibMightyZap.presentPosition(self.portHandler,4)
            self.g_Trgt_Pos_IRR1_1 = self.g_Crrt_Pos_IRR1_1
            self.g_Trgt_Pos_IRR1_2 = self.g_Crrt_Pos_IRR1_2

        # print(g_Crrt_Pos_IRR1_1,g_Crrt_Pos_IRR1_2)41
        if self.g_flgComm_DXL:
            tmp, dxl_comm_result, dxl_error \
                = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID_Axis1, self.ADDR_PRO_PRESENT_POSITION)
            if tmp & 0x80000000:
                tmp = -(((~tmp)&0xffffffff) + 1)
            self.Axis1_CurPos = tmp
            tmp, dxl_comm_result, dxl_error \
                = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID_Axis2, self.ADDR_PRO_PRESENT_POSITION)
            if tmp & 0x80000000:
                tmp = -(((~tmp)&0xffffffff) + 1)
            self.Axis2_CurPos = tmp
            self.Axis1_TarPos = self.Axis1_CurPos
            self.Axis2_TarPos = self.Axis2_CurPos
        time.sleep(0.2)
        while True:
            # Update : position
            if self.g_flgComm_DXL:
                tmp, dxl_comm_result, dxl_error \
                    = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID_Axis1,self.ADDR_PRO_PRESENT_POSITION)
                if tmp & 0x80000000:
                    tmp = -(((~tmp)&0xffffffff) + 1)
                self.Axis1_CurPos = tmp
                tmp, dxl_comm_result, dxl_error \
                    = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID_Axis2,self.ADDR_PRO_PRESENT_POSITION)
                if tmp & 0x80000000:
                    tmp = -(((~tmp) & 0xffffffff) + 1)
                self.Axis2_CurPos = tmp
                # Excution command : Move
                if self.g_flgMove_DXL:
                    dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID_Axis1, self.ADDR_PRO_GOAL_POSITION, self.Axis1_TarPos)
                    dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID_Axis2, self.ADDR_PRO_GOAL_POSITION, self.Axis2_TarPos)
                    self.g_flgMove_DXL = False
                # Excution command : Survo On/Off
                if self.g_flgServo_DXL:
                    self.g_flgServo_DXL = 0
                    if self.g_flgForce_DXL:
                        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_Axis1, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
                        if self.ErrorPrint(dxl_comm_result, dxl_error, self.DXL_ID_Axis1):
                            print("Axis1 : Torque [On]")

                        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_Axis2, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
                        if self.ErrorPrint(dxl_comm_result, dxl_error, self.DXL_ID_Axis2):
                            print("Axis2 : Torque [On]")
                    else:
                        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_Axis1, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
                        if self.ErrorPrint(dxl_comm_result, dxl_error, self.DXL_ID_Axis1):
                            print("Axis1 : Torque [Off]")

                        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_Axis2, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
                        if self.ErrorPrint(dxl_comm_result, dxl_error, self.DXL_ID_Axis1):
                            print("Axis2 : Torque [Off]")
            if self.g_flgComm_DXL:
                # === Get Position
                self.g_Crrt_Pos_IRR1_1 = PythonLibMightyZap.presentPosition(self.portHandler, 0)
                self.g_Crrt_Pos_IRR1_2 = PythonLibMightyZap.presentPosition(self.portHandler, 4)
                # self.stime = time.time()
                # === Get State
                # None
                # === Get Error
                # None
                # === Excute Command ===
                # 1. Move position
                if self.g_flgComm_Move_IRR:
                    PythonLibMightyZap.goalPosition(self.portHandler, 0, self.g_Trgt_Pos_IRR1_1)
                    PythonLibMightyZap.goalPosition(self.portHandler, 4, self.g_Trgt_Pos_IRR1_2)
                    self.g_flgComm_Move_IRR = False
                # 2. Grip_End
                elif self.g_flgForce_IRR:
                    if self.g_flgGripCnt > 2:
                        if (abs(self.g_Crrt_Pos_IRR1_1 - self.g_bfr_Pos_IRR1_1) < 40 or abs(self.g_Crrt_Pos_IRR1_2 - self.g_bfr_Pos_IRR1_2) < 40):
                            self.g_flgForce_IRR = False
                            PythonLibMightyZap.forceEnable(self.portHandler, 0, 0)
                            PythonLibMightyZap.forceEnable(self.portHandler, 4, 0)
                            self.g_Trgt_Pos_IRR1_1 = self.g_Crrt_Pos_IRR1_1
                            self.g_Trgt_Pos_IRR1_2 = self.g_Crrt_Pos_IRR1_2
                            PythonLibMightyZap.goalPosition(self.portHandler, 0, self.g_Trgt_Pos_IRR1_1)
                            PythonLibMightyZap.goalPosition(self.portHandler, 4, self.g_Trgt_Pos_IRR1_2)
                            self.g_flgGripCnt = 0
                            # print("!!F-OFF!! Vel1: [%05d]"%(self.g_Crrt_Pos_IRR1_1 - self.g_bfr_Pos_IRR1_1), "Vel2: [%05d]"%(self.g_Crrt_Pos_IRR1_2 - self.g_bfr_Pos_IRR1_2),"Pos1 : [%05d"%self.g_Crrt_Pos_IRR1_1 ,"/%05d]"%self.g_Trgt_Pos_IRR1_1, "Pos2 : [%05d/"%self.g_Crrt_Pos_IRR1_2, "%05d]"%self.g_Trgt_Pos_IRR1_2)
                    else:
                        self.g_flgGripCnt += 1
                # 3. print
                if abs(self.g_Trgt_Pos_IRR1_1 - self.g_Crrt_Pos_IRR1_1) > 10 and abs(self.g_Trgt_Pos_IRR1_2 - self.g_Crrt_Pos_IRR1_2) > 10:
                    pass
                    # print("Vel1: [%05d]"%(self.g_Crrt_Pos_IRR1_1 - self.g_bfr_Pos_IRR1_1), "Vel2: [%05d]"%(self.g_Crrt_Pos_IRR1_2 - self.g_bfr_Pos_IRR1_2),"Pos1 : [%05d"%self.g_Crrt_Pos_IRR1_1 ,"/%05d]"%self.g_Trgt_Pos_IRR1_1, "Pos2 : [%05d/"%self.g_Crrt_Pos_IRR1_2, "%05d]"%self.g_Trgt_Pos_IRR1_2)

                if self.g_flgGrip_IRR:
                    if self.g_Crrt_Pos_IRR1_1 >= 1400 and self.g_Crrt_Pos_IRR1_2 >= 1400 :
                        PythonLibMightyZap.goalPosition(self.portHandler, 0, 1400)
                        PythonLibMightyZap.goalPosition(self.portHandler, 4, 1400)
                        self.g_flgGrip_IRR = 0
                    elif abs(self.g_Trgt_Pos_IRR1_1 - self.g_Crrt_Pos_IRR1_1) < 50 and abs(self.g_Trgt_Pos_IRR1_2 - self.g_Crrt_Pos_IRR1_2) < 50 :
                        self.g_Trgt_Pos_IRR1_1 = self.g_Crrt_Pos_IRR1_1 + 200
                        self.g_Trgt_Pos_IRR1_2 = self.g_Crrt_Pos_IRR1_2 + 200
                        PythonLibMightyZap.goalPosition(self.portHandler,0, self.g_Trgt_Pos_IRR1_1)
                        PythonLibMightyZap.goalPosition(self.portHandler,4, self.g_Trgt_Pos_IRR1_2)

                self.g_bfr_Pos_IRR1_1 = self.g_Crrt_Pos_IRR1_1
                self.g_bfr_Pos_IRR1_2 = self.g_Crrt_Pos_IRR1_2
                # 2. Force Off Function

    def moveDXL(self):
        self.g_flgMove_DXL = True


class flipper_slave():
    def __init__(self):
        self.g_Cmd = 0
        self.IRR_Grip_pos = 100
        self.IRR_Release_pos = 0

        self.g_move_cnt = 0
        self.g_move_flg = False

        self.DXL_Origin_Axis_1 = 0
        self.DXL_Down_Axis_2 = -62978
        self.DXL_Turn_Axis_1 = 131592
        self.DXL_Up_Axis_2 = -125432
        self.DXL_OffSet_Axis1 = 200
        self.DXL_OffSet_Axis2 = 200
        self.g_Cmd = 0

        self.robot = GripingRobot()

        self.connected = False

    def socket_setting(self):

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(0)

        print '[FS]wait connect client...'
        self.client_socket, self.addr = self.server_socket.accept()
        print('[FS]Connected by', self.addr)
        self.connected = True

        while True:
            time.sleep(0.1)
            data = self.client_socket.recv(1024)
            print('[FS]Received from', self.addr, data)

            if not data:
                print '[FS]Finished comm'
                self.client_socket.close()
                self.server_socket.close()
                print '[FS]Restart Server'

                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_socket.bind((self.HOST, self.PORT))
                self.server_socket.listen(0)

                print '[FS]wait connect client...'
                self.client_socket, self.addr = self.server_socket.accept()
                print('[FS]Connected by', self.addr)
            else:
                if data == '0':
                    self.robot.DXL_TrqDisable()
                elif data == '1':
                    self.robot.DXL_TrqEnable()
                elif data == '2':
                    self.IRR_Grip_pos = 2700
                    self.g_Cmd = 4
                    self.run()
                # elif data == 'q':
                #     break

    def init(self, HOST='127.0.0.1', PORT=9999):
        print('나는 뒤집개다')

        self.HOST = HOST
        self.PORT = PORT
        self.t1 = threading.Thread(target=self.socket_setting)
        self.t1.daemon = True
        self.t1.start()

        self.robot.PortOpenDXL()
        self.robot.start()

        self.robot.DXL_TrqEnable()

    def run(self):
        if self.connected is False:
            print 'Flipper slave does not connected. Please confirm connected.'
        else:
            print('뒤집개 작동한다.')
            self.IRR_Grip_pos = 2700
            self.g_Cmd = 4

            while True:
                time.sleep(0.1)
                if self.g_move_flg:
                    # print(">> Wait ")
                    if abs(self.robot.Axis1_CurPos - self.robot.Axis1_TarPos) < self.DXL_OffSet_Axis1 and abs(self.robot.Axis2_CurPos - self.robot.Axis2_TarPos) < self.DXL_OffSet_Axis2 :
                        if abs(self.robot.g_Trgt_Pos_IRR1_1 - self.robot.g_Crrt_Pos_IRR1_1) < 100 and abs(self.robot.g_Trgt_Pos_IRR1_2 - self.robot.g_Crrt_Pos_IRR1_2) < 100:
                            if self.robot.g_flgForce_IRR == False:
                                self.g_move_flg = False
                    else:
                        pass
                        # print("R Pos Error :", self.robot.Axis1_CurPos - self.robot.Axis1_TarPos, self.robot.Axis2_CurPos - self.robot.Axis2_TarPos , "Mode:", self.g_move_cnt)
                        #else:
                        #    griper1.move()
                elif self.g_move_cnt == 0: # Set Position
                    print(">> Set Robot Position ")
                    self.robot.Axis1_TarPos = self.robot.Axis1_CurPos
                    self.robot.Axis2_TarPos = self.DXL_Up_Axis_2
                    self.robot.moveDXL()
                    self.DXL_OffSet_Axis1 = 10000
                    self.DXL_OffSet_Axis2 = 10000
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 1: #
                    print(">> Move Griper : <<==>> ")
                    self.robot.g_Trgt_Pos_IRR1_1 = self.IRR_Release_pos
                    self.robot.g_Trgt_Pos_IRR1_2 = self.IRR_Release_pos
                    self.robot.moveGriper()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 2:  #
                    print(">> Set Robot Position : to Bottom")
                    self.robot.Axis1_TarPos = self.DXL_Origin_Axis_1
                    self.robot.Axis2_TarPos = self.DXL_Up_Axis_2
                    self.DXL_OffSet_Axis1 = 10000
                    self.DXL_OffSet_Axis2 = 10000
                    self.robot.moveDXL()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 3:  #
                    print(">> Set Wait")
                    self.robot.Axis1_TarPos = self.DXL_Origin_Axis_1
                    self.robot.Axis2_TarPos = self.DXL_Down_Axis_2
                    self.DXL_OffSet_Axis1 = 20000
                    self.DXL_OffSet_Axis2 = 20000
                    self.robot.moveDXL()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 4:
                    print(">> Move Griper : >>==<< ")
                    self.robot.g_Trgt_Pos_IRR1_1 = self.IRR_Grip_pos
                    self.robot.g_Trgt_Pos_IRR1_2 = self.IRR_Grip_pos
                    self.robot.moveGriper()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 5:
                    print(">> Set Robot Position : to Top")
                    self.robot.Axis1_TarPos = self.DXL_Origin_Axis_1
                    self.robot.Axis2_TarPos = self.DXL_Up_Axis_2
                    self.DXL_OffSet_Axis1 = 20000
                    self.DXL_OffSet_Axis2 = 20000
                    self.robot.moveDXL()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 6:
                    print(">> Set Robot Position : to left")
                    self.robot.Axis1_TarPos = self.DXL_Turn_Axis_1
                    self.robot.Axis2_TarPos = self.DXL_Up_Axis_2
                    self.DXL_OffSet_Axis1 = 20000
                    self.DXL_OffSet_Axis2 = 20000
                    self.robot.moveDXL()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 7:
                    print(">> Set Robot Position : to Down")
                    self.robot.Axis1_TarPos = self.DXL_Turn_Axis_1
                    self.robot.Axis2_TarPos = self.DXL_Down_Axis_2
                    self.DXL_OffSet_Axis1 = 20000
                    self.DXL_OffSet_Axis2 = 20000
                    self.robot.moveDXL()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 8:
                    print(">> Move Griper : <<==>> ")
                    self.robot.g_Trgt_Pos_IRR1_1 = self.IRR_Release_pos
                    self.robot.g_Trgt_Pos_IRR1_2 = self.IRR_Release_pos
                    self.robot.moveGriper()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 9:
                    print(">> Set Robot Position : to Down")
                    self.robot.Axis1_TarPos = self.DXL_Turn_Axis_1
                    self.robot.Axis2_TarPos = self.DXL_Up_Axis_2
                    self.DXL_OffSet_Axis1 = 20000
                    self.DXL_OffSet_Axis2 = 20000
                    self.robot.moveDXL()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 10:
                    print(">> Set Robot Position : to Down")
                    self.robot.Axis1_TarPos = self.DXL_Origin_Axis_1
                    self.robot.Axis2_TarPos = self.DXL_Up_Axis_2
                    self.DXL_OffSet_Axis1 = 20000
                    self.DXL_OffSet_Axis2 = 20000
                    self.robot.moveDXL()
                    self.g_move_cnt += 1
                    self.g_move_flg = True
                elif self.g_move_cnt == 11:
                    self.DXL_OffSet_Axis1 = 500
                    self.DXL_OffSet_Axis2 = 500
                    self.g_move_cnt = 0
                    self.g_Cmd = 0
                    self.g_move_flg = False
                    break

    def stop(self):
        self.robot.DXL_TrqDisable()
        print('뒤집개 꺼진다.')