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
print('Waiting for connection')
print('...connected from:',ADDR)
sensor = qmc.QMC5883L()
sensor.calibration = [[  1.03194204e+00,  -5.83440263e-02,  -6.59472052e+03],
                      [ -5.83440263e-02,   1.10656882e+00,   4.03742821e+02],
                      [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]

while True:

    tcpDataSock,addr=tcpSerSock.accept()
    #print("accepted")
    try:
        m=sensor.get_bearing()
        
    except:
        m="none"
    try:
    #print(m)
        RawVoltage = adc.read_adc(0,gain = GAIN)
        Voltage = round(100*RawVoltage*4.096/(2**15))/100
        Volts=str(Voltage)+'V'
        print(Volts)
    except:
        Volts="NA"
    
    Message = str(m)+','+Volts+'\r\n'
    #Message = str(math.degrees(math.atan2(y,x)))+'\r\n'
    tcpDataSock.send(Message.encode('UTF-8'))
    

    

