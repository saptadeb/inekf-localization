% Plot raw and processed IMU data through the MadgwickAHRS algorithm 
%
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-18

close all; clear; clc; 

% Beautiful colors
blue      = [0, 0.4470, 0.7410];
orange    = [0.8500, 0.3250, 0.0980];
yellow    = [0.9290, 0.6940, 0.1250];
purple    = [0.4940, 0.1840, 0.5560];
green     = [0.4660, 0.6740, 0.1880];
lightblue = [0.3010, 0.7450, 0.9330];
red       = [0.6350, 0.0780, 0.1840];

% Import data
addpath([cd, '/../data'])
data = load('data_2013-01-10.mat');

% Plot data for Gyroscope and Accelerometer
figure(1);
subplot(2,1,1); hold on;
plot(data.imu.timestamp - data.imu.timestamp(1), data.imu.gyro_x, 'color', red);
plot(data.imu.timestamp - data.imu.timestamp(1), data.imu.gyro_y, 'color', green);
plot(data.imu.timestamp - data.imu.timestamp(1), data.imu.gyro_z, 'color', blue);
legend({'X', 'Y', 'Z'}, 'Location', 'Southeast', 'Interpreter', 'latex')
xlabel('Time ($s$)', 'Interpreter', 'latex');
ylabel('Angular rate ($\mathrm{rad}/s$)', 'Interpreter', 'latex');
title('\textbf{Gyroscope}', 'Interpreter', 'latex');
axis tight
subplot(2,1,2); hold on;
plot(data.imu.timestamp - data.imu.timestamp(1), data.imu.accel_x, 'color', red);
plot(data.imu.timestamp - data.imu.timestamp(1), data.imu.accel_y, 'color', green);
plot(data.imu.timestamp - data.imu.timestamp(1), data.imu.accel_z, 'color', blue);
legend({'X', 'Y', 'Z'}, 'Location', 'Southeast', 'Interpreter', 'latex')
xlabel('Time ($s$)', 'Interpreter', 'latex');
ylabel('Acceleration ($g$)', 'Interpreter', 'latex');
title('\textbf{Accelerometer}', 'Interpreter', 'latex');
axis tight

% Madgwick's algorithm
AHRS = MadgwickAHRS('Beta', 0.1);
quaternion = zeros(length(data.imu.timestamp), 4);
quaternion(1,:) = [1 0 0 0];
for i = 1: length(data.imu.timestamp)-1
    gyro = [data.imu.gyro_x(i);
            data.imu.gyro_y(i);
            data.imu.gyro_z(i)];
    accel = [data.imu.accel_x(i);
             data.imu.accel_y(i);
             data.imu.accel_z(i)];
    dt = data.imu.timestamp(i+1) - data.imu.timestamp(i);
    AHRS.UpdateIMU(gyro, accel, dt);
    quaternion(i+1, :) = AHRS.Quaternion;
end

% Plot algorithm output as Euler angles
% euler = quatern2euler(quaternConj(quaternion)); % use conjugate for sensor frame relative to Earth.
euler = quat2eul(quaternConj(quaternion), 'XYZ');

figure(2); hold on;
plot(data.imu.timestamp - data.imu.timestamp(1), wrapToPi(euler(:,1) + pi), 'color', red);
% plot(data.imu.timestamp - data.imu.timestamp(1), euler(:,1), 'color', red);
plot(data.imu.timestamp - data.imu.timestamp(1), euler(:,2), 'color', green);
plot(data.imu.timestamp - data.imu.timestamp(1), euler(:,3), 'color', blue);
title('Euler angles', 'Interpreter', 'latex');
xlabel('Time ($s$)', 'Interpreter', 'latex');
ylabel('Angle ($rad$)', 'Interpreter', 'latex');
legend({'$\phi$', '$\theta$', '$\psi$'}, 'Interpreter', 'latex');