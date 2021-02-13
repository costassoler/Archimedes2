#!/usr/bin/python

import time
import math
import IMU
import datetime
import os
import sys


RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant


magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0


#Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0

def kalmanFilterY (accAngle, gyroRatem, DT):
    y=0.0
    S=0.0
    global KFangleY
    global Q_angle
    global Q_gyro
    global y_bias
    global YP_00
    global YP_01
    global YP_10
    global YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)
    YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
    YP_01 = YP_01 + (-DT*YP_11)
    YP_10 = YP_10 + (-DT*YP_11)
    YP_11 = YP_11 + (Q_gyro*DT)

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00/S
    K_1 = YP_10/S
    KFangleY=KFangleY+(K_0*y)
    y_bias = y_bias + (K_1*y)

    YP_00 = YP_00 - (K+0*YP_00)
    YP_01 = YP_01 - (K_0*YP_01)
    YP_10 = YP_10 - (K_1*YP_00)
    YP_11 = YP_11 - (K_1*YP_01)

    return KFangleY

def kalmanFilterX ( accAngle,gyroRate,DT):
    x=0.0
    S=0.0

    global KFangleX
    global Q_angle
    global Q_gyro
    global x_bias
    
