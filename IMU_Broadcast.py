from BerryIMU import *
import numpy as np
import csv
gZ=0
gY=0

while True:
    try:
        with open('calibrationdata.csv', newline='') as csvfile:
            data = np.array(list(csv.reader(csvfile)))

        magXmin = float(data[0][0])
        magXmax = float(data[0][1])
        magYmin = float(data[0][2])
        magYmax = float(data[0][3])
        magZmin = float(data[0][4])
        magZmax = float(data[0][5])
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        MAGx -= (magXmin+magXmax)/2
        MAGy -= (magYmin+magYmax)/2
        MAGz -= (magZmin+magZmax)/2

        accXnorm = ACCx/((ACCx*ACCx+ACCy*ACCy+ACCz*ACCz)**0.5)
        accYnorm = ACCy/((ACCx*ACCx+ACCy*ACCy+ACCz*ACCz)**0.5)

        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm/math.cos(pitch))

        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
                
        H = (180*math.atan2(-magYcomp,magXcomp)/M_PI)-90

        if H<0:
            H+=360
               
        newHeading =  np.array([H,pitch])
        newHeading.tofile('heading.csv',sep=',')
        print(H)
    except:
        continue
