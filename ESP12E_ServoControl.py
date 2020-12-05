from socket import *
import smbus
import time
from time import ctime
import os
import Cytron27Aug2019 as c
import pigpio

pi=pigpio.pi()
pi.set_mode(14,pigpio.OUTPUT)
pi.set_servo_pulsewidth(14,1500)


HOST=''
PORT=21568
BUFSIZE=1024
ADDR=(HOST,PORT)
tcpSerSock=socket(AF_INET,SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)
print('Waiting for connection')
tcpCliSock,addr=tcpSerSock.accept()
print('...connected from:',ADDR)
tcpSerSock.settimeout(0.5) #Socket reader moves on after waiting for a new message for 1 second

Dat="DevOff"
angle=0

#WiFi Control Setup:
s = socket(AF_INET,SOCK_STREAM)
s.bind(('',8095))
s.listen(5)
s.settimeout(0.5)

while True:
    try:
        
        '''START Disconnect Safety Override'''      
        try:
            #reads new commands from socket
            tcpCliSock,addr=tcpSerSock.accept()
            Dat=tcpCliSock.recv(BUFSIZE).decode("utf-8")
            print(Dat)
        
        except:
            #catches any exception (most likely timeout after 0.5 second) that prevents successful command read.'''
            Dat = Dat
            
        '''END Disconnect Safety Override'''
        
        try:
            "DO CONTROL STUFF HERE"
            if (Dat=="DevOn"):
                angle=180
            if (Dat=="DevOff"):
                angle=0

            pi.set_servo_pulsewidth(14,500+(angle*2000/180))
            
                
        except:
            pass
        try:
            client,addr = s.accept()
            message = str(angle)+'*'
            client.send(message.encode('UTF-8'))
            print(message)
        except:
            continue
    except KeyboardInterrupt:
        break
tcpSerSock.close()
                            
