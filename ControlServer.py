from socket import *
from time import ctime
import Cytron27Aug2019 as c
import pigpio
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
scale = .7
while True:
    print("while true")
    try:
        tcpCliSock,addr=tcpSerSock.accept()
        Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
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
            print(i,j,k,float(data[3]))

            c.L(L*scale)
            c.R(R*scale)
            c.LV(V*scale)
            c.RV(V*scale)
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
            
