#!/usr/bin/env python
# -- coding: utf-8 --

import math
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from mpl_toolkits.mplot3d import Axes3D
from ikpy.utils import plot
import numpy as np

from tool import *


#***********************#
#    ikpy 패키지 실험     #
#***********************#



test_link = Chain(name='test_arm' , links = [
    OriginLink(),
    URDFLink(
        name="first_link",
        origin_translation=[0,0,0],
        origin_orientation=[0,0,0],
        rotation=[0,0,1],
    ),
    URDFLink(
        name="second_link",
        origin_translation=[0,0,7.2],
        origin_orientation=[math.pi/2,0,0],
        rotation=[1,0,0],
        bounds=(-math.pi, 0.000001)
    ),
    URDFLink(
        name="fixed_link",
        origin_translation=[0,0,26.3],
        origin_orientation=[-math.pi/2,0,0],
        joint_type="fixed"
    ),
    URDFLink(
        name="third_link",
        origin_translation=[0,0,2.7],
        origin_orientation=[0,0,0],
        rotation=[1,0,0],
        bounds=(-math.pi/2, math.pi),
    ),
    URDFLink(
        name="fourth_link",
        origin_translation=[0,0,27.5],
        origin_orientation=[0,0,0],
        rotation=[1,0,0],
        bounds=(-math.pi/2,0.0001)
    ),
    URDFLink(
        name="fifth_link",
        origin_translation=[0,0,7.16],
        origin_orientation=[0,0,0],
        rotation=[0,0,1],
        bounds=(-math.pi*(5/6),math.pi*(5/6))
    ),
    URDFLink(
        name="sixth_link",
        origin_translation=[0,0,0],
        origin_orientation=[0,0,0],
        joint_type="fixed"
    )
])


ax = plt.figure().add_subplot(111, projection='3d')


angle=test_link.inverse_kinematics(target_position=[0,50,4],target_orientation=[[1,0,0],[0,-1,0],[0,0,-1]],orientation_mode="all")
print(angle)
print(xm_rad_to_position(angle[1]))
print(xm_rad_to_position(angle[2]))
print(xm_rad_to_position(-angle[2]))
print(xm_rad_to_position(-angle[4]))
print(xm_rad_to_position(angle[4]))
print(ax_rad_to_position(angle[5]))
print(ax_rad_to_position(angle[6]))

# test_link.plot([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0], ax)
test_link.plot(test_link.inverse_kinematics(target_position=[0,50,4],target_orientation=[[1,0,0],[0,-1,0],[0,0,-1]],orientation_mode="all"), ax) #각도는 순서대로 (OriginLink, First, Second, Third, Fourth), OriginLink는 0 라디안 고정
plt.show()
