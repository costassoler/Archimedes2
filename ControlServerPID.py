from socket import *
from time import ctime
import Cytron27Aug2019 as c
import pigpio
import py_qmc5883l as qmc
import time
#servo setup:
pi=pigpio.pi()
pi.set_mode(23,pigpio.OUTPUT)
pi.set_servo_pulsewidth(23,1500)

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
L = "none"
R = "none"
#PID SETUP START
Kp=.5
#Ki=0
#Kd=0
AutoAdjust=0
#errorT = 0
sensor = qmc.QMC5883L()
sensor.calibration = [[  1.03194204e+00,  -5.83440263e-02,  -6.59472052e+03],
                      [ -5.83440263e-02,   1.10656882e+00,   4.03742821e+02],
                      [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]
mprev=sensor.get_bearing()
AutoOn=0 #Autopilot throttle value is zero
AutoPilotArm="AutoOff" #Autopilot is disarmed
while True:
    
    try:
        tcpCliSock,addr=tcpSerSock.accept()
        Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")  
        
        try:
            data=Dat.split(",")
            i=float(data[0])
            j=float(data[1])
            k=float(data[2])
            AutoPilotArm = data[4]
            m=sensor.get_bearing()
            #print(m)
            
                
            #PID Section START
            if(i==0) and (j==0) and (AutoOn==0) and (AutoPilotArm=="AutoOn"):
                c.L(0)
                c.R(0)

                time.sleep(.5)

                mOrig=mprev
                #print(mOrig)

            if(i==0) and (j==0):
                if(mOrig<90) and (m>270):
                    m-=360
                if(mOrig>270) and (m<90):
                    m+=360
                error = m - mOrig
                #print(error)
                
                
                if (abs(error)<5):
                    error=0
                
                '''    
                errorT += error
                if (abs(errorT)>30):
                    errorT=30
                '''
                    
                AutoAdjust = error*Kp #+ errorT*Ki
                AutoOn=1
                if (abs(AutoAdjust)>30):
                    AutoAdjust=30
                    
            if (i>0) or (j>0) or (AutoPilotArm=="AutoOff"):
                #errorT=0
                error=0
                AutoAdjust=0
                AutoOn=0
            #PID Section END
                
            if(abs(i)<2):
                i=0
            if(abs(j)<2):
                j=0
            if(abs(k)<20):
                k=0
            #print(AutoAdjust)

            L=i-AutoAdjust
            R=j+AutoAdjust
            V=k
            
        

            c.L(L)
            c.R(R)
            c.LV(V)
            c.RV(V)
            cam=float(data[3])
            angle=1000+cam*1000/180
            if(angle<1200):
                angle=1200
            if(angle>2000):
                angle=2000
            pi.set_servo_pulsewidth(23,angle)

            ##PID SECTION:

            mprev=m
            
        

                
        except:
            continue
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
