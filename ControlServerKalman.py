from socket import *
from time import ctime
import os
import Cytron27Aug2019 as c
import pigpio
from BerryIMU import *
import numpy as np
import math
import csv
#Yaw rate setup:
gZ=0
gY=0
a = datetime.datetime.now()

#servo setup:
pi=pigpio.pi()
pi.set_mode(23,pigpio.OUTPUT)
pi.set_servo_pulsewidth(23,1500)
print("servo setup")


HOST=''
PORT=21567
BUFSIZE=1024
ADDR=(HOST,PORT)
tcpSerSock=socket(AF_INET,SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)
print('Waiting for connection')
tcpCliSock,addr=tcpSerSock.accept()
print('...connected from:',ADDR)
tcpSerSock.settimeout(0.5) #Socket reader moves on after waiting for a new message for 1 second
scale = .7

L=0
R=0
V=0
cam=100
n=0
#COMPASS CALIBRATION LOOP
MAGx = IMU.readMAGx()
MAGy = IMU.readMAGy()
MAGz = IMU.readMAGz()

magXmin = MAGx
magXmax = MAGx
magYmin = MAGy
magYmax = MAGy
magZmin = MAGz
magZmax = MAGz

with open('calibrationdata.csv', newline='') as csvfile:
    data = np.array(list(csv.reader(csvfile)))

magXmin = float(data[0][0])
magXmax = int(float(data[0][1]))
magYmin = int(float(data[0][2]))
magYmax = int(float(data[0][3]))
magZmin = int(float(data[0][4]))
magZmax = int(float(data[0][5]))
if(magXmin==999999):
    magXmin = MAGx
if(magXmax==999999):
    magXmax = MAGx
if(magYmin==999999):
    magYmin=MAGy
if(magYmax==999999):
    magYmax=MAGy
if(magZmin==999999):
    magZmin=MAGz
if(magZmax==999999):
    magZmax=MAGz
while True:
    try:     
        try:
            #reads new commands from socket
            tcpCliSock,addr=tcpSerSock.accept()
            Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
            data=Dat.split(",")
            l=data[4]
            print(l)
            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()
            if (MAGx<magXmin):
                magXmin = MAGx
            if (MAGx>magXmax):
                magXmax = MAGx

            if (MAGy<magYmin):
                magYmin = MAGy
            if (MAGy>magYmax):
                magYmax = MAGy

            if (MAGz<magZmin):
                magZmin = MAGz
            if (MAGz>magZmax):
                magZmax = MAGz
            
            if (l=='AutoOn'):
                print("Calibration Complete")
                break
        except Exception as e:
            print(e)
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            Dat="0,0,0,"+str(cam)+",AutoOff"
    except KeyboardInterrupt:
        break
calibration_array =  np.array([magXmin,magXmax,magYmin,magYmax,magZmin,magZmax])
calibration_array.tofile('calibrationdata.csv',sep=',')

#MODEL C CONTROL LOOP
while True:
    try:
        '''START Disconnect Safety Override'''      
        try:
            #reads new commands from socket
            tcpCliSock,addr=tcpSerSock.accept()
            Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
            data=Dat.split(",")
            n=0
        
        except:
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            Dat="0,0,0,"+str(cam)+",AutoOff"
            
        '''END Disconnect Safety Override'''
        try:
            
            l=data[4]
            if(l=='AutoOn'):
                ACCx = IMU.readACCx()
                ACCy = IMU.readACCy()
                ACCz = IMU.readACCz()
                MAGx = IMU.readMAGx()
                MAGy = IMU.readMAGy()
                MAGz = IMU.readMAGz()

                MAGx -= (magXmin+magXmax)/2
                MAGy -= (magYmin+magYmax)/2
                MAGz -= (magZmin+magZmax)/2

                accXnorm = ACCx/math.sqrt(ACCx*ACCx+ACCy*ACCy+ACCz*ACCz)
                accYnorm = ACCy/math.sqrt(ACCx*ACCx+ACCy*ACCy+ACCz*ACCz)

                pitch = math.asin(accXnorm)
                roll = -math.asin(accYnorm/math.cos(pitch))

                magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
                
                tiltCompensatedHeading = (180*math.atan2(-magYcomp,magXcomp)/M_PI)-90
                if tiltCompensatedHeading<0:
                    tiltCompensatedHeading+=360
                print(tiltCompensatedHeading)
        except Exception as e:
            print(e)
        
        try:
            i=float(data[0])
            j=float(data[1])
            k=float(data[2])
           
            

            if(abs(i)<5):
                i=0
            if(abs(j)<5):
                j=0
            if(abs(k)<5):
                k=0
            L=i
            R=j
            V=k
            print(i,j,k,float(data[3]))
            cam=float(data[3])
            angle=1000+cam*1000/180
            if(angle<1200):
                angle=1200
            if(angle>2000):
                angle=2000
            pi.set_servo_pulsewidth(23,angle)
            

            c.L(L*scale)
            c.R(R*scale)
            c.LV(V*scale)
            c.RV(V*scale)
                
        except Exception as e:
            print(e)
            continue
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
