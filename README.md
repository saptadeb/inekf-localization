# Invariant  Extended  Kalman  Filtering  for  Robot  Localization using  IMU  and  GPS

NA 568 Final Project Team 16 - Saptadeep Debnath, Anthony Liang, Gaurav Manda, Sunbochen Tang, Hao Zhou

This project aims to implement an In-EKF based localization system and compare it against a particle filter based localization system and a GPS-alone dataset. We will use the [UM North Campus Long-Term Vision and LIDAR dataset](http://robots.engin.umich.edu/nclt/), an autonomy dataset for robotics research collected on the University of Michigan North Campus. It consists of data from several sensors including planar lidar, omnidirectional camera, IMU, and GPS.

## Goals
- Research done by [A. Barrau](https://ieeexplore.ieee.org/document/7402522), [M. Barczyk](https://ieeexplore.ieee.org/document/7081772?section=abstract) and [R. Hartley](https://arxiv.org/abs/1805.10410) were used as the baseline for the results anticipated from the project.
- Develop an In-EKF filter model for pose estimation on the IMU sensor data from The UM North Campus Long-Term Vision and LIDAR Dataset and using GPS sensor data to implement a correction model.
- Develop an EKF based pose estimation model using IMU and GPS (for correction) data.
- Compare the proposed In-EKF based localization system with the EKF based localization, only GPS data and the ground truth poses provided by the dataset.

## Dependencies

1. Download sensor data and groundtruth.csv from the [NCLT dataset](http://robots.engin.umich.edu/nclt/) for a date, save all csv files in one folder named by the date, and store the folder in `/data`.

2. Run the script `/utils/read_data_and_convert_to_mat.py` 

## Running the code
1. `/src/LIEKF_example.m` runs the Left-Invarriant EKF on the NCLT, and compares with ground truth.
2. `/src/madgwick_example.m` runs the Madgwick algorithm.
3. `/src/EKF_example.m` runs the 

`/src/LIEKF_example.m` and `/src/EKF_example.m` produces three plots; planned robot trajectory compared with the ground truth, comparison of the computed euler angles with the ground truth and Mahalanobis distances for the predicted robot states.

## Results

Add plots and stuff...... make gifs from the vids 

Check the [proposal](https://github.com/team16-mobrob-w20/inekf-localization/blob/master/EECS568_Team16_proposal.pdf) and the [final report]() for more details on implementation. //todo add the links

## Team Members
- [Saptadeep Debnath](https://www.linkedin.com/in/saptadeep-deb/) (saptadeb@umich.edu)
- [Anthony Liang](https://www.linkedin.com/in/anthony-liang/) (aliangdw@umich.edu)
- [Gaurav Manda](https://www.linkedin.com/in/gaurav-manda-8a7844103/) (gmanda@umich.edu)
- [Sunbochen Tang](https://www.linkedin.com/in/sunbochen-tang-4773b3152/) (tangsun@umich.edu)
- [Hao Zhou](https://www.linkedin.com/in/hao-zhou-96a85b180/) (zhh@umich.edu)
