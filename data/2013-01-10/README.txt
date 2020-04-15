***************************************************************
*                                                             *
*                 NCLT Sensor Data                            *
*                                                             *
***************************************************************

Changelog:

   08/14/2018         Arash Ushani           Created file

------------------------------------------------------------------

The contents of this tarball include sensor data for NCLT.

The format of some of these files is described in our paper, available here:
http://robots.engin.umich.edu/nclt/nclt.pdf. These files include:

    gps.csv
    gps_rtk.csv
    gps_rtk_err.csv
    ms25.csv
    ms25_euler.csv
    odometry_mu.csv
    odometry_mu_100hz.csv
    odometry_cov.csv
    odometry_cov_100hz.csv

Additionally, since the publication of the paper, we have included additional data,
which we describe here.

kvh.csv

    This CSV file contains the data from the KVH single-axis fiber optic gyro. Each
    line of this file contains the timestamp followed by the heading in radians.

wheels.csv

    This CSV file contains wheel speed data. Each line of this file contains the
    timestamp, followed by the left wheel speed and the right wheel speed in m/s.
