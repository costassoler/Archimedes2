#!/usr/bin/env/python3
from socket import *
from time import ctime
import os
import Cytron27Aug2019 as c
import pigpio
import numpy as np
import math
import csv
#IMU setup:
gZ=0
gY=0
imuConnection="NA"
C=0

tiltCompensatedHeading = 0
holdHeading=0
#servo setup:
pi=pigpio.pi()
pi.set_mode(23,pigpio.OUTPUT)
pi.set_servo_pulsewidth(23,1500)
print("servo setup")
magXmin=999999
magYmin=999999
magZmin=999999
magXmax=999999
magYmax=999999
magZmax=999999
try:
    from BerryIMU import *
    a = datetime.datetime.now()
    print(a)
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()
    imuConnection = "connected"

    magXmin = MAGx
    magXmax = MAGx
    magYmin = MAGy
    magYmax = MAGy
    magZmin = MAGz
    magZmax = MAGz

    with open('calibrationdata.csv', newline='') as csvfile:
        data = np.array(list(csv.reader(csvfile)))

    magXmin = float(data[0][0])
    magXmax = float(data[0][1])
    magYmin = float(data[0][2])
    magYmax = float(data[0][3])
    magZmin = float(data[0][4])
    magZmax = float(data[0][5])
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
    
except:
    print("No IMU detected")
    imuConnection = "not connected"
    holdHeading=0
    tiltCompensatedHeading=0
HOST=''
PORT=21567
BUFSIZE=2048
ADDR=(HOST,PORT)
tcpSerSock=socket(AF_INET,SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)
print('Waiting for connection')
tcpCliSock,addr=tcpSerSock.accept()
print('...connected from:',ADDR)
tcpSerSock.settimeout(0.5) #Socket reader moves on after waiting for a new message for 1 second
scale = 1
L=0
R=0
V=0
cam=100
n=0
m=0
hcGain = 1

#COMPASS CALIBRATION LOOP
while True:
    try:     
        try:
            print("calibrating...")
            tcpCliSock,addr=tcpSerSock.accept()
            Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
            data=Dat.split(",")
            l=data[4]
            
            if (l=='AutoOn') or (imuConnection == "not connected"):
                print("Calibration Complete")
                break
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

            holdHeading = tiltCompensatedHeading
            with open('heading.csv', newline='') as csvfile:
                data = np.array(list(csv.reader(csvfile)))
            tiltCompensatedHeading= float(data[0][0])
            holdHeading = tiltCompensatedHeading
        except Exception as e:
            print(e)
    except KeyboardInterrupt:
        break

calibration_array =  np.array([magXmin,magXmax,magYmin,magYmax,magZmin,magZmax])
calibration_array.tofile('calibrationdata.csv',sep=',')


hcGain=1
#MODEL C CONTROL LOOP
while True:
    try:
        '''START Disconnect Safety Override'''      
        try:
            #reads new commands from socket
            tcpCliSock,addr=tcpSerSock.accept()
            Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
            lC=0
            rC=0
            n=0
        except:
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            n+=1
            if(n>5):
                Dat="0,0,0,"+str(cam)+",AutoOff"
            C=0
        '''END Disconnect Safety Override'''     
          
        try:
            if(imuConnection=="connected") and (abs(i-j)<15):
                with open('heading.csv', newline='') as csvfile:
                    headingData = np.array(list(csv.reader(csvfile)))
                tiltCompensatedHeading= float(headingData[0][0])
                print(tiltCompensatedHeading)
                C = tiltCompensatedHeading - holdHeading
                print()
            if(l=='AutoOn') and (abs(i-j)<15):
                if (C>180):
                    C-=360
                if(C<-180):
                    C+=360
                
                if(C>30):
                    C=30
                if(C<-30):
                    C=-30
                m=0    
            if(abs(i-j)>=15) or (l=='AutoOff'):
                holdHeading = tiltCompensatedHeading
                C=0
                m=0 
        except Exception as e:
            m+=1
            if (m>=5):
                C=0
            print(e)
        
        try:
            data=Dat.split(",")
            i=float(data[0])
            j=float(data[1])
            k=float(data[2])
            cam=float(data[3])
            l=data[4]
            if(abs(i-j)>=15) or (l=='AutoOff'):
                holdHeading = tiltCompensatedHeading
                C=0
            L=i-hcGain*C
            R=j+hcGain*C
            V=k
            
            if(abs(L)<5):
                L=0
            if(abs(R)<5):
                R=0
            if(abs(V)<5):
                V=0
                
            angle=1000+cam*1000/180
            if(angle<1200):
                angle=1200
            if(angle>2000):
                angle=2000
            pi.set_servo_pulsewidth(23,angle)
            c.R(R*scale)
            c.L(L*scale)
            c.LV(V*scale)
            c.RV(V*scale)
            print(C)
            #time.sleep(0.1)
            #print(tiltCompensatedHeading)
            
        except Exception as e:
            print(e)
            continue
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
