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
z_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
ZP_00 = 0.0
ZP_01 = 0.0
ZP_10 = 0.0
ZP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0
KFangleZ = 0.0

def kalmanFilterY (accAngle, gyroRate, DT):
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

    YP_00 = YP_00 - (K_0*YP_00)
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
    global XP_00
    global XP_01
    global XP_10
    global XP_11

    KFangleX = KFangleX + DT*(gyroRate-x_bias)

    XP_00 = XP_00 + (-DT*(XP_10+XP_01)+Q_angle*DT)
    XP_01 = XP_01 + (-DT*XP_11)
    XP_10 = XP_10 + (-DT*XP_11)
    XP_11 = XP_11 + (Q_gyro*DT)

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00/S
    K_1 = XP_10/S

    KFangleX = KFangleX + (K_0*x)
    x_bias = x_bias + (K_1*x)

    XP_00 = XP_00 - (K_0*XP_00)
    XP_01 = XP_01 - (K_0*XP_01)
    XP_10 = XP_10 - (K_1*XP_00)
    XP_11 = XP_11 - (K_1*XP_01)

    return KFangleX

def kalmanFilterZ(magAngle,gyroRate,DT):
    z = 0.0
    S = 0.0
    global KFangleZ
    global Q_angle
    global Q_gyro
    global z_bias
    global ZP_00
    global ZP_01
    global ZP_10
    global ZP_11
    
    KFangleZ = KFangleZ + DT*(gyroRate-z_bias)
    ZP_00 = ZP_00 + (-DT*(ZP_10+ZP_01)+Q_angle*DT)
    ZP_01 = ZP_01 + (-DT*ZP_11)
    ZP_10 = ZP_10 + (-DT*ZP_11)
    ZP_11 = ZP_11 + (Q_gyro*DT)

    z = magAngle - KFangleZ
    S = XP_00 + R_angle
    K_0 = ZP_00/S
    K_1 = ZP_10/S

    KFangleZ = KFangleZ + (K_0*z)
    z_bias = z_bias + (K_1*z)

    ZP_00 = ZP_00 - (K_0*ZP_00)
    ZP_01 = ZP_01 - (K_0*ZP_01)
    ZP_10 = ZP_10 - (K_1*ZP_00)
    ZP_11 = ZP_11 - (K_1*ZP_01)
    return KFangleZ

    
    
IMU.detectIMU()
if(IMU.BerryIMUversion==99):
    print("No BerryIMU found. Quitting")
    sys.exit()
IMU.initIMU()

gyroXangle=0.0
gyroYangle=0.0
gyroZangle=0.0
CFangleX = 0.0
CFangleY = 0.0
kalmanX = 0.0
kalmanY=0.0
a = datetime.datetime.now()

