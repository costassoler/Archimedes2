from BerryIMU import *
import numpy as np
import ahrs

def quaternion_to_euler(x, y, z,w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 
while True:
    try:
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        

        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()

        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()
        #a = acc_data.T
        a = np.array([[ACCx],[ACCy],[ACCz]],dtype=float)
        g = np.array([[GYRx],[GYRy],[GYRz]],dtype=float)
        m = np.array([[MAGx],[MAGy],[MAGz]],dtype=float)
        acc_data = a.T
        gyro_data = g.T
        mag_data = m.T

        attitude = ahrs.filters.Madgwick(acc=acc_data, gyr=gyro_data, mag=mag_data, gain=0.1, frequency=2.0)

        #print(quaternion_to_euler(attitude.Q[0][0],attitude.Q[0][1],attitude.Q[0][2],attitude.Q[0][3]))
        print(quaternion_to_euler(attitude.Q[0][0],attitude.Q[0][1],attitude.Q[0][2],attitude.Q[0][3])[0])
        #attitude.updateIMU
    except KeyboardInterrupt:
        break
