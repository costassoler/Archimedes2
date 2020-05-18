from socket import *
from time import ctime
import os
import Cytron27Aug2019 as c
import pigpio

#IMU Setup:
from mpu6050 import mpu6050
calN = 20

accelAvg = 0
gyroz = 0

for i in range (0,calN):
    try:
        IMU   = mpu6050(0x68)
        accel = IMU.get_accel_data()
        gyro  = IMU.get_gyro_data()

        accelT+=(accel['x']**2+accel['y']**2+accel['z']**2)**.5
    except:
        break
accelAvg = accelT/calN

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
V        = 0
n        = 0
bearingC = 0
udC      = 0
while True:
    print("while true")
    try:
        
        '''START Disconnect Safety Override'''      
        try:
            #reads new commands from socket
            tcpCliSock,addr=tcpSerSock.accept()
            Dat            =tcpCliSock.recv(BUFSIZE).decode("utf-8")
            n              =0
            data=Dat.split(",")
            i   =   float(data[0])
            j   =   float(data[1])
            k   =   float(data[2])
            cam  =float(data[3])
            AutoPilotArm = data[4]
        
        except:
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            Dat="0,0,0,125"
            
        '''END Disconnect Safety Override'''

        '''START IMU Readout'''
        try:
            N+=1
            IMU   = mpu6050(0x68)
            accel = IMU.get_accel_data()
            accelT=(accel['x']**2+accel['y']**2+accel['z']**2)**.5
            measurement = accel['z']
            gyro  = IMU.get_gyro_data()['z']
        
    
            if (abs(gyro)<5):
                bearingC=0
            if(gyro>20):
                gyro = 20
            if(gyro<-20):
                gyro=-20
            bearingC = gyro
        
        
            accZ+= (accelAvg-accelT)/accelAvg
        

            if(N%10==0):
            
                udC = P*accZ//10
                print("here")
        
                if (udC>20):
                    udC=20
                if(udC<-20):
                    udC=-20
                if(abs(udC)<2):
                    udC=0
            
                print(udC)
                accZ = 0
            
        except:
            bearingC = 0
            udC      = 0


        '''END IMU Readout'''


        
        '''START Data Processing'''
        try:
            
            if(abs(i)<5):
                i=0
            if(abs(j)<5):
                j=0
            if(abs(k)<5):
                k=0
            if (imuOn==0):
                bearingC=0
                udC = 0
            L=i#+bearingC
            R=j#-bearingC
            V=k+udC
            print(i,j,k,float(data[3]))
            
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
        '''END Data Processing'''
        
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
