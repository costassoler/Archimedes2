from socket import *
from time import ctime
import os
import Cytron27Aug2019 as c
import pigpio
import numpy as np
import math
import csv
#Yaw rate setup:
gZ=0
gY=0

#servo setup:
pi=pigpio.pi()
pi.set_mode(23,pigpio.OUTPUT)
pi.set_servo_pulsewidth(23,1500)
print("servo setup")


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
hcGain = 0.5
#COMPASS CALIBRATION LOOP

    
while True:
    try:     
        try:
            print("calibrating...")
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

            if (l=='AutoOn') or (imuConnection == "not connected"):
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
            i=float(data[0])
            j=float(data[1])
            k=float(data[2])
            cam=float(data[3])
            l=data[4]
            
        except:
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            i=0
            j=0
            k=0
            cam=cam
            l="AutoOff"
            
            lC = 0
            rC = 0
            
        '''END Disconnect Safety Override'''
        try:    
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
                    
                if(holdHeading<90) and (tiltCompensatedHeading>270):
                    tiltCompensatedHeading-=360
                if(holdHeading>270) and (tiltCompensatedHeading<90):
                    tiltCompensatedHeading+=360
                    
                C = tiltCompensatedHeading - holdHeading
                lC = hcGain*C
                rC = -hcGain*C
                
                if(lC>20):
                    lC=20
                if(lC<-20):
                    lC=-20
                if(rC>20):
                    rC=20
                if(rC<-20):
                    rC=-20
                
            if(l=='AutoOff') or (abs(i-j)>15):
                holdHeading = tiltCompensatedHeading
                lC = 0
                rC = 0
                    
        except Exception as e:
            print(e)
        
        try:
            L=i+lC
            R=j+rC
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
            
            c.L(L*scale)
            c.R(R*scale)
            c.LV(V*scale)
            c.RV(V*scale)
            print(L,R,V,l)
                
        except Exception as e:
            print(e)
            continue
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
