from socket import *
from time import ctime
import os
import Cytron27Aug2019 as c
import pigpio
from BerryIMU import *
import numpy as np
#Yaw rate setup:
gZ=0
gY=0
a = datetime.datetime.now()

#servo setup:
pi=pigpio.pi()
pi.set_mode(23,pigpio.OUTPUT)
pi.set_servo_pulsewidth(23,1500)
print("servo setup")

#Eth0Status = os.popen("sudo ifconfig eth0").read()

#print(Eth0Status)

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

while True:
    print("while true")
    try:
        
        '''START Disconnect Safety Override'''      
        try:
            #reads new commands from socket
            tcpCliSock,addr=tcpSerSock.accept()
            Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
            n=0
        
        except:
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            Dat="0,0,0,"+str(cam)+",AutoOff"
            
        '''END Disconnect Safety Override'''
        try:
            ACCx = IMU.readACCx()
            ACCy = IMU.readACCy()
            ACCz = IMU.readACCz()
            GYRx = IMU.readGYRx()
            GYRy = IMU.readGYRy()
            GYRz = IMU.readGYRz()
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            rate_gyr_x =  GYRx * G_GAIN
            rate_gyr_y =  GYRy * G_GAIN
            rate_gyr_z =  GYRz * G_GAIN
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP
            AccXangle = (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
            AccYangle = (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
            if AccYangle > 90:
                AccYangle -= 270.0
            else:
                AccYangle +=90.0
            kalmanY = kalmanFilterY(AccYangle,rate_gyr_y,LP)
            kalmanX = kalmanFilterX(AccXangle,rate_gyr_x,LP)
            dY = rate_gyr_y*LP
            dZ = rate_gyr_z*LP
            if (np.abs(GYRy)<40):
                dY=0
            if (np.abs(GYRz)<40):
                dZ=0
            gZ+=dZ
            gY+=dY
            heading=gZ*np.cos(kalmanX/RAD_TO_DEG)+gY*np.sin(kalmanX/RAD_TO_DEG)
            print(heading)
        except:
            print("IMU or kalman failure")
        
        try:
            data=Dat.split(",")
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
            #print(i,j,k,float(data[3]))
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
                
        except:
            continue
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
