#!/usr/bin/env python
# -- coding: utf-8 --

import os, sys
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from dynamixel_sdk import *
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.robotis_def import *
from manipulator.msg import *
from manipulator_description import Manipulator
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np

import sys, tty, termios

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


#*************************AX-12A(PROTOCOL_VERSION 1.0)*****************************#

# Control table address
AX_ADDR_TORQUE_ENABLE          = 24 # 토크 활성화(1)/비활성화(0)
AX_ADDR_CW_COMPLIANCE_MARGIN   = 26 # 시계방향 : Goal Position을 도달했다고 판단하는 Margin값, 예를 들어 Goal Position이 30이고 Margin값이 2라면 28~32에 도달하면 goal position에 도달한것으로 판단함
AX_ADDR_CCW_COMPLIANCE_MARGIN  = 27 # 반시계 방향 : ```
AX_ADDR_CW_COMPLIANCE_SLOPE    = 28 # 시계방향 : 가속/김속하는 시간
AX_ADDR_CCW_COMPLIANCE_SLOPE   = 29 # 반시계방향 : ```
AX_ADDR_GOAL_POSITION          = 30 # 목표 각도
AX_ADDR_MOVING_SPEED           = 32 # 목표 속도
AX_ADDR_PRESENT_POSITION       = 36 # 현재 각도
AX_ADDR_PRESENT_SPEED          = 38 # 현재 속도
AX_ADDR_PRESENT_LOAD           = 40
AX_ADDR_MOVING                 = 46
AX_ADDR_PUNCH                  = 48 # 모터에 가하는 최소 전류 -> 다르게 생각하면 최소 속도라고 할 수 있을 듯

AX_PROTOCOL_VERSION = 1.0

AX_DXL_ID = [5,6]#[5,6,7]

BAUDRATE = 115200

AX_TORQUE_ENABLE = 1
AX_TORQUE_DISABLE = 0

AX_CW_COMPLIANCE_MARGIN = 0 #실제로 설정하려는 값
AX_CCW_COMPLIANCE_MARGIN = 0
AX_CW_COMPLIANCE_SLOPE = 64
AX_CCW_COMPLIANCE_SLOPE = 64

DEVICENAME = '/dev/ttyUSB0'


port_handler = PortHandler(DEVICENAME)
ax_packet_handler = PacketHandler(AX_PROTOCOL_VERSION)

#**********************************************************************************#


#**********************XM430-W350-R(PROTOCOL_VERSION 2.0)**************************#

# Control table address

XM_ADDR_TORQUE_ENABLE           = 64
XM_ADDR_VELOCITY_I_GAIN         = 76
XM_ADDR_VELOCITY_P_GAIN         = 78
XM_ADDR_POTISION_D_GAIN         = 80
XM_ADDR_POSITION_I_GAIN         = 82
XM_ADDR_POSITION_P_GAIN         = 84
XM_ADDR_FEEDFORWARD_2ND_GAIN    = 88
XM_ADDR_FEEDFORWARD_1ST_GAIN    = 90
XM_ADDR_PROFILE_ACCELERATION    = 108
XM_ADDR_PROFILE_VELOCITY        = 112
XM_ADDR_GOAL_POSITION           = 116
XM_ADDR_MOVING                  = 122
XM_ADDR_MOVING_STATUS           = 123
XM_ADDR_PRESENT_POSITION        = 132

XM_PROTOCOL_VERSION_1 = 1.0
XM_PROTOCOL_VERSION_2 = 2.0

XM_DXL_ID_P1 = [3,4]
XM_DXL_ID_P2 = [0,1,2]


XM_TORQUE_ENABLE = 1
XM_TORQUE_DISABLE = 0

XM_PROFILE_VELOCITY = 30

xm_packet_handler_p1 = PacketHandler(XM_PROTOCOL_VERSION_1)
xm_packet_handler_p2 = PacketHandler(XM_PROTOCOL_VERSION_2)


#**********************************************************************************#

class MotorControlHub:

    def __init__(self):

        self.set_pos = SyncSetPosition()
        self.set_ax_speed = AXSyncSetMovingSpeed()

        self.manipulator = Manipulator()
        
        self.target_position = Point()
        

        #테스트용(이후에 지워야함)
        self.target_position.x = 10
        self.target_position.y = 20
        self.target_position.z = 20
        

        #아래방향 바라봄
        self.orientation_matrix = [
            [1,0,0],
            [0,-1,0],
            [0,0,-1]
        ] 

        self.gripper_position = 512

        self.target_position_flag = False
        
        self.set_pos.ax_id = AX_DXL_ID
        self.set_pos.xm_id_p1 = XM_DXL_ID_P1
        self.set_pos.xm_id_p2 = XM_DXL_ID_P2

        self.set_pos.ax_position = [512,512]#[512, 512, 512]
        self.set_pos.xm_position_p1 = [2048+1024,2048-1024]
        self.set_pos.xm_position_p2 = [2048,2048+100,2048-100]#[2048, 2048, 2048, 2048, 2048]

        self.set_ax_speed.id = AX_DXL_ID
        self.set_ax_speed.speed = [100,100]#[100, 100, 100]
        
        rospy.Subscriber('target_position', Point, self.set_target_position_callback, queue_size=1)
        rospy.Subscriber('grip',Bool, self.gripper_callback, queue_size=1)

        rospy.Subscriber('set_position',SyncSetPosition, self.set_goal_pos_callback, queue_size=1)
        rospy.Subscriber('set_ax_speed',AXSyncSetMovingSpeed, self.set_ax_moving_speed_callback, queue_size=1)

        self.pos_pub = rospy.Publisher('present_position', SyncSetPosition, queue_size=1)
        self.ax_speed_pub = rospy.Publisher('present_ax_speed', AXSyncSetMovingSpeed, queue_size=1)


    def set_target_position_callback(self,data):
        self.target_position = data
        self.target_position_flag = True


    def gripper_callback(self,msg:Bool):
        if msg.data is True:
            self.gripper_position = 100
        else:
            self.gripper_position = 512


    def set_goal_pos_callback(self,data):
        self.set_pos = data


    def set_ax_moving_speed_callback(self,data):
        self.set_ax_speed = data


    def set_goal_pos(self,data:SyncSetPosition):
        for idx in range(len(data.ax_id)):
            # print("Set Goal AX_Position of ID %s = %s" % (data.ax_id[idx], data.ax_position[idx]))
            ax_packet_handler.write2ByteTxRx(port_handler,data.ax_id[idx], AX_ADDR_GOAL_POSITION, data.ax_position[idx])

        for idx in range(len(data.xm_id_p1)):
            # print("Set Goal XM_Position of ID %s = %s" % (data.xm_id[idx], data.xm_position[idx]))
            xm_packet_handler_p1.write4ByteTxRx(port_handler,data.xm_id_p1[idx], XM_ADDR_GOAL_POSITION, data.xm_position_p1[idx])

        for idx in range(len(data.xm_id_p2)):
            # print("Set Goal XM_Position of ID %s = %s" % (data.xm_id[idx], data.xm_position[idx]))
            xm_packet_handler_p2.write4ByteTxRx(port_handler,data.xm_id_p2[idx], XM_ADDR_GOAL_POSITION, data.xm_position_p2[idx])


    def set_ax_moving_speed(self,data:AXSyncSetMovingSpeed): #저장된 speed는 이 함수를 통해 입력된다
        for idx in range(len(data.id)) :
            # print("Set AX_Moving Speed of ID %s = %s" % (data.id[idx], data.speed[idx]))
            ax_packet_handler.write2ByteTxRx(port_handler, data.id[idx], AX_ADDR_MOVING_SPEED, data.speed[idx])


    def present_position_callback(self):
        present_position = SyncSetPosition()
        present_position.ax_id = AX_DXL_ID
        present_position.xm_id_p1 = XM_DXL_ID_P1
        present_position.xm_position_p2= XM_DXL_ID_P2
        present_position.ax_position = []
        present_position.xm_position_p1 = []
        present_position.xm_position_p2 = []

        for id in AX_DXL_ID:
            dxl_present_position, dxl_comm_result, dxl_error = ax_packet_handler.read2ByteTxRx(port_handler, id, AX_ADDR_PRESENT_POSITION)
            present_position.ax_position.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return
            
        for id in XM_DXL_ID_P1:
            dxl_present_position, dxl_comm_result, dxl_error = xm_packet_handler_p1.read4ByteTxRx(port_handler, id, XM_ADDR_PRESENT_POSITION)
            present_position.xm_position_p1.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return

        for id in XM_DXL_ID_P2:
            dxl_present_position, dxl_comm_result, dxl_error = xm_packet_handler_p2.read4ByteTxRx(port_handler, id, XM_ADDR_PRESENT_POSITION)
            present_position.xm_position_p2.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return

        self.pos_pub.publish(present_position)

    
    def present_speed_callback(self) : #현재 speed를 publish 해줌
        present_speed = AXSyncSetMovingSpeed()

        present_speed.id = AX_DXL_ID
        present_speed.speed = []

        for id in AX_DXL_ID:
            dxl_present_speed, dxl_comm_result, dxl_error = ax_packet_handler.read2ByteTxRx(port_handler, id, AX_ADDR_PRESENT_SPEED)
            present_speed.speed.append(dxl_present_speed%1024)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return

        self.ax_speed_pub.publish(present_speed)

    
    def set_target_position(self):
        target_pos = [self.target_position.x, self.target_position.y, self.target_position.z]
        motor_angles = self.manipulator.manipulator_link.inverse_kinematics(target_position=target_pos,target_orientation=self.orientation_matrix,orientation_mode="all")
        if self.check_inverse_kinematics(target_pos,motor_angles) is False:
            print("도달할 수 없는 타겟")
            return
        
        motor_angles = np.append(np.array(motor_angles[1:7]), [self.gripper_position])

        self.manipulator.set_position(motor_angles)
    
    def check_inverse_kinematics(self, target_position, motor_angles):
        return self.distance_target_point(np.transpose(np.array(self.manipulator.manipulator_link.forward_kinematics(motor_angles)[:3,3:]))[0], target_position)


    def distance_target_point(self, p1, p2):
        distance = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)**(1/2)
        # print("distance: ",distance)
        if distance > 1:
            return False
        else:
            return True



def main():
    rospy.init_node('motor_control_hub')
 

    try:
        port_handler.openPort()
        print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    try:
        port_handler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    for id in AX_DXL_ID :
        dxl_comm_result, dxl_error = ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % ax_packet_handler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % ax_packet_handler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print(f"DYNAMIXEL(ID : {id}) has been successfully connected")
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CW_COMPLIANCE_MARGIN, AX_CW_COMPLIANCE_MARGIN) #초기 margin 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CCW_COMPLIANCE_MARGIN, AX_CCW_COMPLIANCE_MARGIN) #초기 margin 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CW_COMPLIANCE_SLOPE, AX_CW_COMPLIANCE_SLOPE) #초기 slope 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CCW_COMPLIANCE_SLOPE, AX_CCW_COMPLIANCE_SLOPE) #초기 slope 설정
            ax_packet_handler.write2ByteTxRx(port_handler, id, AX_ADDR_MOVING_SPEED, 100) #초기 속도 설정

    for id in XM_DXL_ID_P1 :
        dxl_comm_result, dxl_error = xm_packet_handler_p1.write1ByteTxRx(port_handler, id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % xm_packet_handler_p1.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % xm_packet_handler_p1.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            xm_packet_handler_p1.write4ByteTxRx(port_handler, id, XM_ADDR_PROFILE_VELOCITY, XM_PROFILE_VELOCITY)
        print(f"DYNAMIXEL(ID : {id}) has been successfully connected")

    for id in XM_DXL_ID_P2 :
        dxl_comm_result, dxl_error = xm_packet_handler_p2.write1ByteTxRx(port_handler, id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % xm_packet_handler_p2.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % xm_packet_handler_p2.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            xm_packet_handler_p2.write4ByteTxRx(port_handler, id, XM_ADDR_PROFILE_VELOCITY, XM_PROFILE_VELOCITY)
        print(f"DYNAMIXEL(ID : {id}) has been successfully connected")

    
    print("Ready to get & set Position.")

    ############################################################################################################
    #  여기까지는 dynamixel 기본 설정
    ############################################################################################################


    data_hub = MotorControlHub()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        data_hub.set_goal_pos(data_hub.set_pos)
        data_hub.set_ax_moving_speed(data_hub.set_ax_speed)

        data_hub.present_position_callback()
        data_hub.present_speed_callback()

        # if data_hub.target_position_flag is True:
        data_hub.set_target_position()

        rate.sleep()



if __name__ == '__main__':
    main()