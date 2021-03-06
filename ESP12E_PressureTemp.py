from socket import *
import numpy as np
import smbus
import time
#import py_qmc58831
import py_qmc5883l as qmc
import math
import Adafruit_ADS1x15
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1 #This sets the voltage range to 4.096V per 2**15
#from mpu6050 import mpu6050
#sensor = mpu6050(0x68)
HOST=''
PORT=52849
BUFSIZE=1024
ADDR=(HOST,PORT)
tcpSerSock=socket(AF_INET,SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)

#External HW:
s = socket(AF_INET,SOCK_STREAM)
s.bind(('',8095))
s.listen(5)
s.settimeout(0.5)

print('Waiting for connection')
print('...connected from:',ADDR)
content="NA,NA"
while True:

    tcpDataSock,addr=tcpSerSock.accept()
    #print("accepted")
    try:
        sensor = qmc.QMC5883L()
        sensor.calibration = [[  1.03194204e+00,  -5.83440263e-02,  -6.59472052e+03],
                              [ -5.83440263e-02,   1.10656882e+00,   4.03742821e+02],
                              [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]
        m=str(sensor.get_bearing())
        
    except:
        m='none'
        pass
    try:
    #print(m)
        RawVoltage = adc.read_adc(0,gain = GAIN)
        Voltage = round(100*RawVoltage*4.096/(2**15))/100
        Volts=str(Voltage)+'V'
        print(Volts)
    except:
        Volts="NA"
    try:
        client,addr = s.accept()
        while True:
            content = str(client.recv(32).decode('UTF-8'))
            #content = client.read_until(b'*')
            if len(content)==0:
                content="NA,NA"
            else:
                print(content)
            client.close()
            #print("Connection closed")
    except Exception as e:
        if(e==socket.timeout):
            content="NA,NA"
        print(e)
        pass  
    try:
        Message = m+','+Volts+','+content+'\r\n'
        tcpDataSock.send(Message.encode('UTF-8'))
    
    except Exception as e:
        print(e)
        pass
    

