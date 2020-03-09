from socket import *
from time import ctime
import Cytron27Aug2019 as c
import pigpio
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
while True:
    
    try:
        tcpCliSock,addr=tcpSerSock.accept()
        Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
        try:
            data=Dat.split(",")
            i=float(data[0])
            j=float(data[1])
            k=float(data[2])

            if(i<20) and (i>-20):
                i=0
            if(j<20) and (j>-20):
                j=0
            if(k<20) and (k>-20):
                k=0
            L=i
            R=j
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

                
        except:
            continue
    except KeyboardInterrupt:
        break
tcpSerSock.close()
            
