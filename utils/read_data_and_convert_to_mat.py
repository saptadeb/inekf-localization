"""
Read IMU and GPS .csv data and convert to .mat

Author: Hao Zhou <zhh@umich.edu>
Date:   2020-04-14
"""

import numpy as np
import scipy.io as sio

date = '2013-01-10'
directory = '../data/' + date + '/'
gpsConsumerGradeFile = 'gps.csv'
gpsRTKFile = 'gps_rtk.csv'
imuFile = 'ms25.csv'
imuEulerFile = 'ms25_euler.csv'
groundtruthFile = 'groundtruth.csv'
outputFile = '../data/data_' + date + '.mat'

def main():
    """ 
    GPS CSV File Format:
        Field | Description                               | Unit
        0     | UTIME of the GPS fix                      | microsecond
        1     | Fix mode                                  |
        2     | Number of satellites used in the fix      |
        3     | Latitude                                  | rad
        4     | Longitude                                 | rad
        5     | Altitude                                  | m
        6     | Track                                     | m
        7     | Speed                                     | m/s
    
    IMU CSV File Format:
        Field | Description                               | Unit
        0     | UTIME of the measurements                 | microsecond
        1-3   | 3-DOF magnetic field strength vector      | Gauss
        4-6   | 3-DOF acceleration vector                 | m/s2
        7-9   | 3-DOF angular rate (roll, pitch, heading) | rad/s

    IMU EULER CSV File Format:
        Field | Description                               | Unit
        0     | UTIME of the measurements                 | microsecond
        1-3   | 3-DOF angular rate (roll, pitch, heading) | rad  

    GROUNDTRUTH CSV File Format:
        Field | Description                               | Unit
        0     | UTIME of the measurements                 | microsecond
        1-3   | x, y, z                                   | m
        4-6   | roll, pitch, heading                      | rad
    """
    gpsCG     = np.loadtxt(directory + gpsConsumerGradeFile, delimiter = ",")
    gpsCGTime = gpsCG[:, 0] * 1e-6
    latCG     = gpsCG[:, 3]
    lngCG     = gpsCG[:, 4]
    altCG     = gpsCG[:, 5]

    gpsRTK     = np.loadtxt(directory + gpsRTKFile, delimiter = ",")
    gpsRTKTime = gpsRTK[:, 0] * 1e-6
    latRTK     = gpsRTK[:, 3]
    lngRTK     = gpsRTK[:, 4]
    altRTK     = gpsRTK[:, 5]

    imu     = np.loadtxt(directory + imuFile, delimiter = ",")
    imuTime = imu[:, 0] * 1e-6
    # magX    = imu[:, 1]
    # magY    = imu[:, 2]
    # magZ    = imu[:, 3]
    accelX  = imu[:, 4]
    accelY  = imu[:, 5]
    accelZ  = imu[:, 6]
    gyroX   = imu[:, 7]
    gyroY   = imu[:, 8]
    gyroZ   = imu[:, 9]

    imuEuler = np.loadtxt(directory + imuEulerFile, delimiter = ",")
    imuEulerTime = imuEuler[:, 0] * 1e-6
    imuRoll      = imuEuler[:, 1]
    imuPitch     = imuEuler[:, 2]
    imuHeading   = imuEuler[:, 3]

    gt      = np.loadtxt(directory + groundtruthFile, delimiter = ",")
    # NED (North, East Down)
    gtTime  = gt[:, 0] * 1e-6
    x       = gt[:, 1]
    y       = gt[:, 2]
    z       = gt[:, 3]
    roll    = gt[:, 4]
    pitch   = gt[:, 5]
    heading = gt[:, 6]

    data = {'gps_cg': {'timestamp': gpsCGTime, 'latitude': latCG, 'longitude': lngCG, 'altitude': altCG},
            'gps_rtk': {'timestamp': gpsRTKTime, 'latitude': latRTK, 'longitude': lngRTK, 'altitude': altRTK},
            'imu': {'timestamp': imuTime, 'accel_x': accelX, 'accel_y': accelY, 'accel_z': accelZ,
                    'gyro_x': gyroX, 'gyro_y': gyroY, 'gyro_z': gyroZ},
            'imu_euler': {'timestamp': imuEulerTime, 'roll': imuRoll, 'pitch': imuPitch, 'heading': imuHeading},
            'ground_truth': {'timestamp': gtTime, 'x': x, 'y': y, 'z': z,
                             'roll': roll, 'pitch': pitch, 'heading': heading}}

    sio.savemat(outputFile, data, oned_as='column') 

    return 0

if __name__ == '__main__':
    main()
