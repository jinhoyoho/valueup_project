#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from manipulator.msg import *

import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from std_msgs.msg import Int16
#from ikpy import plot_utils
import numpy as np

from tool import *

AX_DXL_ID = [5,6,7]

XM_DXL_ID_P1 = [3,4]
XM_DXL_ID_P2 = [0,1,2]


class Manipulator:

    def __init__(self):

        self.link1_length = 7.2
        self.link2_length = 26.3
        self.link3_length = 27.5
        self.link4_length = 7.16
        self.link5_length = 4.3

        self.fixed_frame_length = 2.7

        self.ax_position = []
        self.xm_position_p1 = []
        self.xm_position_p2 = []
        self.ax_speed = []

        self.position_pub_data = SyncSetPosition()

        self.ax_speed_pub_data = AXSyncSetMovingSpeed()


        self.position_pub = rospy.Publisher('set_position', SyncSetPosition, queue_size=1)
        self.ax_speed_pub = rospy.Publisher('set_ax_speed', AXSyncSetMovingSpeed, queue_size=1)
        
        rospy.Subscriber('present_position', SyncSetPosition, self.position_callback, queue_size=1)
        rospy.Subscriber('present_ax_speed', AXSyncSetMovingSpeed, self.ax_speed_callback, queue_size=1)

        self.manipulator_link = Chain(name='manipulator', links=[
            OriginLink(),
            URDFLink(
                name="first_link",
                origin_translation=[0,0,0],
                origin_orientation=[0,0,0],
                rotation=[0,0,1],
            ),
            URDFLink(
                name="second_link",
                origin_translation=[0,0,self.link1_length],
                origin_orientation=[math.pi/2,0,0],
                rotation=[1,0,0],
                bounds=(-math.pi, 0.000001)
            ),
            URDFLink(
                name="fixed_link",
                origin_translation=[0,0,self.link2_length],
                origin_orientation=[-math.pi/2,0,0],
                joint_type="fixed"
            ),
            URDFLink(
                name="third_link",
                origin_translation=[0,0,self.fixed_frame_length],
                origin_orientation=[0,0,0],
                rotation=[1,0,0],
                bounds=(-math.pi/2, math.pi)
            ),
            URDFLink(
                name="fourth_link",
                origin_translation=[0,0,self.link3_length],
                origin_orientation=[0,0,0],
                rotation=[1,0,0],
                 bounds=(-math.pi/2,0.0001)
            ),
            URDFLink(
                name="fifth_link",
                origin_translation=[0,0,self.link4_length],
                origin_orientation=[0,0,0],
                rotation=[0,0,1],
                bounds=(-math.pi*(5/6),math.pi*(5/6))
            ),
            URDFLink(
                name="sixth_link",
                origin_translation=[0,0,self.link5_length],
                origin_orientation=[0,0,0],
                joint_type="fixed"
            )
        ])

    
    def position_callback(self, msg:SyncSetPosition):
        self.ax_position = msg.ax_position
        self.xm_position_p1 = msg.xm_position_p1
        self.xm_position_p2 = msg.xm_position_p2
        print("XM_P2_position: ", self.xm_position_p2)
        print("XM_P1_position: ", self.xm_position_p1)
        print("AX_position   : ", self.ax_position)

    
    def ax_speed_callback(self, msg:AXSyncSetMovingSpeed):
        self.ax_speed = msg.speed


    def set_position(self, angles): # xm = [0,1,2,3,4] ax = [5,6,7] (1,2) , (3,4)는 서로 반대로 모터를 돌려야함 [angle1,~, angle6]을 인수로 받으면 됨
        self.position_pub_data.ax_id = AX_DXL_ID
        self.position_pub_data.xm_id_p1 = XM_DXL_ID_P1
        self.position_pub_data.xm_id_p2 = XM_DXL_ID_P2

        self.position_pub_data.ax_position = []
        self.position_pub_data.xm_position_p1 = []
        self.position_pub_data.xm_position_p2 = []

        self.position_pub_data.xm_position_p2.append(xm_rad_to_position(angles[0]))
        self.position_pub_data.xm_position_p2.append(xm_rad_to_position(angles[1])+100)
        self.position_pub_data.xm_position_p2.append(xm_rad_to_position(-angles[1])-100)
        self.position_pub_data.xm_position_p1.append(xm_rad_to_position(-angles[3]))
        self.position_pub_data.xm_position_p1.append(xm_rad_to_position(angles[3]))

        self.position_pub_data.ax_position.append(ax_rad_to_position(angles[4]))
        self.position_pub_data.ax_position.append(ax_rad_to_position(angles[5]))
        self.position_pub_data.ax_position.append(int(angles[6]))
        
        # print(self.position_pub_data.xm_position_p2, self.position_pub_data.xm_position_p1 , self.position_pub_data.ax_position)

        self.position_pub.publish(self.position_pub_data)



