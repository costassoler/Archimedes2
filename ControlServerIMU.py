from socket import *
from time import ctime
import os
import Cytron27Aug2019 as c
import pigpio
import numpy as np

#IMU Setup:
from mpu6050 import mpu6050
IMU   = mpu6050(0x68)

print(IMU.get_gyro_data()['z'])
calN = 20
N=0
P = 40
bearingC=0

'''
accelAvg = 12
cInt=0
cInt=0.05
gyro=0
cDer = 1

accelT = 0
accZ = 0


for i in range (0,calN):
    try:
        
        IMU   = mpu6050(0x68)
        accel = IMU.get_accel_data()
        gyro  = IMU.get_gyro_data()
        
        accelT+=(accel['x']**2+accel['y']**2+accel['z']**2)**.5
        
        accelAvg = accelT/calN
        
    except:
        continue
'''

#servo setup:
pi=pigpio.pi()
pi.set_mode(23,pigpio.OUTPUT)
pi.set_servo_pulsewidth(23,1500)


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
V        = 0
cam      = 100
N        = 0
bearingC = 0
udC      = 0
MotorArm="AutoOff"
while True:
    
    try:
        
        '''START Disconnect Safety Override'''      
        try:
            #reads new commands from socket
            tcpCliSock,addr=tcpSerSock.accept()
            Dat            =tcpCliSock.recv(BUFSIZE).decode("utf-8")
                    
        except:
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            Dat = "0,0,0,"+str(cam)+",AutoOff"
            
            
        '''END Disconnect Safety Override'''

        '''START IMU Readout'''
        try:
            if(MotorArm=="AutoOn"):
                N+=1
                gyro  = (IMU.get_gyro_data()['z']+IMU.get_gyro_data()['y'])*0.707
                bearingC+=gyro
                
                     
                if(N==5):
                    bearingAvg=bearingC//5
                    print(bearingAvg)
                    bearingC=0
                    N=0
                                  
        except Exception as inst:
            #print(type(inst))
            #print(inst.args)
            
            bearingAvg=0
            
        '''END IMU Readout'''


        
        '''START Data Processing'''
        try:
            data=Dat.split(",")
            i   =   float(data[0])
            j   =   float(data[1])
            k   =   float(data[2])
            cam  =float(data[3])
            MotorArm = str(data[4])
            
            if(abs(i)<5):
                i=0
            if(abs(j)<5):
                j=0
            if(abs(k)<5):
                k=0
        
            if(abs(i-j)>10) or (MotorArm != "AutoOn"):
                bearingAvg=0
                bearingC=0
                N=0
                
            L=i#+bearingAvg
            R=j#-bearingAvg
            V=k
            #print(bearingC)
            #print(i,j,k,float(data[3]))
            
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
                
        except Exception as inst:
            print(type(inst))
            print(inst.args)
            continue
        '''END Data Processing'''
        
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
