from BerryIMU import *
import numpy as np
gZ=0
gY=0
while True:
    try:
        #with open('calibrationdata.csv', newline='') as csvfile:
            #data = np.array(list(csv.reader(csvfile)))
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        

        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()

        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        # Calibrate Compass:
        MAGx -= (magXmin+magYmax)/2
        MAGy -= (magYmin + magYmax) /2
        MAGz -= (magZmin + magZmax) /2

        b = datetime.datetime.now() - a
        a = datetime.datetime.now()

        LP = b.microseconds/(1000000*1.0)
        
        #Convert gyro vals to degrees per second:
        rate_gyr_x =  GYRx * G_GAIN
        rate_gyr_y =  GYRy * G_GAIN
        rate_gyr_z =  GYRz * G_GAIN

        #Convert values to -180 and +180
        gyroXangle+=rate_gyr_x*LP
        gyroYangle+=rate_gyr_y*LP
        gyroZangle+=rate_gyr_z*LP

        #Convert accelerometer vals to degrees:
        AccXangle = (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
        AccYangle = (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
        MagZangle = (math.atan2(MAGy,-MAGx)*RAD_TO_DEG)

        #Convert values to -180 and +180:
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle +=90.0

        #Complementary filter used to combine acc and gyro values:
        CFangleX = AA*(CFangleX+rate_gyr_x*LP) + (1-AA)*AccXangle
        CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

        #Kalman filter used to combine accel and gyro vals:
        kalmanY = kalmanFilterY(AccYangle,rate_gyr_y,LP)
        kalmanX = kalmanFilterX(AccXangle,rate_gyr_x,LP)
        

        #calculate heading:
        heading=180*math.atan2(MAGy,MAGx)/M_PI

        #only have our heading between 0 and 360:
        if heading <0:
            heading += 360

        # TILT COMPENSATED HEADING #

        #normalize Accel raw values:
        accXnorm = ACCx/math.sqrt(ACCx*ACCx+ACCy*ACCy+ACCz*ACCz)
        accYnorm = ACCy/math.sqrt(ACCx*ACCx+ACCy*ACCy+ACCz*ACCz)
        accZnorm = ACCz/math.sqrt(ACCx*ACCx+ACCy*ACCy+ACCz*ACCz)

        #Calculate pitch and roll:
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm/math.cos(pitch))
        
        MAGx -= (-2574+257)/2
        MAGy -= (-585+2213)/2
        MAGz -= (-1772+1475)/2
        
        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
        rawHeading = 180*math.atan2(-MAGy,MAGx)/M_PI

        tiltCompensatedHeading = 180*math.atan2(-magYcomp,magXcomp)/M_PI
        if tiltCompensatedHeading<0:
            tiltCompensatedHeading+=360
        #print(rawHeading)
        #print(gyroZangle)
        #print("X: ",kalmanX)

        #UNFILTERED MODELC YAW
        #modelcYaw = np.cos(kalmanX)*gyroZangle+np.sin(kalmanX)*gyroYangle
        
        #LOW PASS CUTOFF:
        dY = rate_gyr_y*LP
        dZ = rate_gyr_z*LP
        if (np.abs(dY)<0.5):
            dY=0
        if (np.abs(dZ)<0.5):
            dZ=0
        gZ+=dZ
        gY+=dY
        #print(gZ*np.cos(kalmanX/RAD_TO_DEG)+gY*np.sin(kalmanX/RAD_TO_DEG))
        time.sleep(0.1)
        #print(kalmanY*np.cos(kalmanX/RAD_TO_DEG))
        print(tiltCompensatedHeading)
        

        calibration_array =  np.array([magXmin,magXmax,magYmin,magYmax,magZmin,magZmax])
        calibration_array.tofile('calibrationdata.csv',sep=',')
    except Exception as e:
        print(e)
        continue
