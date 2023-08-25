#!/usr/bin/env python
# -- coding: utf-8 --

import math
import numpy as np


def ax_rad_to_position(rad):
    y_axis_position = 512
    add_position = round(rad*(180/math.pi)/(300/1024))
    position = y_axis_position+add_position
    if(position > 1023):
        print("overposition: ",position)
        position = 1023
    elif(position < 0) :
        position = 0
    return position


def xm_rad_to_position(rad):
    y_axis_position = 2048
    add_position = round(rad*(180/math.pi)/(360/4096))
    position = y_axis_position+add_position
    if(position > 4095):
        position = 4095
    elif(position < 0) :
        position = 0
    return position


def ax_position_to_rad(position):
    y_axis_position = 512
    theta = (position-y_axis_position)/(1024/300)
    rad = theta*(math.pi/180)
    return rad


def xm_position_to_rad(position):
    y_axis_position = 2048
    theta = (position-y_axis_position)/(4096/360)
    rad = theta*(math.pi/180)
    return rad


