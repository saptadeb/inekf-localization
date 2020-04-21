clear, clc, close all

% Beautiful colors
blue      = [0, 0.4470, 0.7410];
orange    = [0.8500, 0.3250, 0.0980];
yellow    = [0.9290, 0.6940, 0.1250];
purple    = [0.4940, 0.1840, 0.5560];
green     = [0.4660, 0.6740, 0.1880];
lightblue = [0.3010, 0.7450, 0.9330];
red       = [0.6350, 0.0780, 0.1840];

% Load data
addpath([cd, '/../data'])
data = load('data_2013-01-10.mat');
[xGpsCg,yGpsCg,zGpsCg] = latlngalt2xyz(data.gps_cg.latitude, data.gps_cg.longitude, data.gps_cg.altitude);

% Run LIEKF
[filteredData, filteredDataStatistics] = EKF_run();

figure(1); hold on
plot(filteredData(:,5), filteredData(:,6), 'color', red)
plot(xGpsCg, yGpsCg, 'color', green)
plot(filteredData(:,11), filteredData(:,12), 'color', blue)
axis equal
title('\textbf{Robot location}', 'Interpreter', 'latex');
legend({'EKF', 'Consumer grade GPS', 'Ground truth'}, 'Location','Best', 'Interpreter', 'latex')
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')

figure(2); hold on
plot(filteredData(:,1) - filteredData(1,1), wrapToPi(filteredData(:,2) + pi), 'color', red);
plot(filteredData(:,1) - filteredData(1,1), filteredData(:,3), 'color', green);
plot(filteredData(:,1) - filteredData(1,1), filteredData(:,4), 'color', blue);
plot(filteredData(:,1) - filteredData(1,1), filteredData(:,8), 'color', orange, 'linestyle', '--');
plot(filteredData(:,1) - filteredData(1,1), filteredData(:,9), 'color', yellow, 'linestyle', '--');
plot(filteredData(:,1) - filteredData(1,1), filteredData(:,10), 'color', purple, 'linestyle', '--');
legend({'$\phi_{\mathrm{EKF}}$', '$\theta_{\mathrm{EKF}}$', '$\psi_{\mathrm{EKF}}$', ...
        '$\phi_{\mathrm{GT}}$', '$\theta_{\mathrm{GT}}$', '$\psi_{\mathrm{GT}}$', }, 'Interpreter', 'latex');
axis([0 filteredData(end,1) - filteredData(1,1) -4 4]);
title('\textbf{Euler angles}', 'Interpreter', 'latex');
xlabel('Time ($s$)', 'Interpreter', 'latex')
ylabel('Angle ($rad$)', 'Interpreter', 'latex');

figure(3);
subplot(4,1,1)
plot(filteredDataStatistics(:,5), 'linewidth', 2, 'color', blue)
hold on; grid on
ylabel('$x$', 'fontsize', 14, 'Interpreter', 'latex')
plot(filteredDataStatistics(:,10),'r', 'linewidth', 2, 'color', red)
plot(-1*filteredDataStatistics(:,10),'r', 'linewidth', 2, 'color', red)
legend({'Deviation from Ground Truth','3rd Sigma Contour'}, 'location', 'Best')
axis([0 length(filteredDataStatistics) -500 500]);

subplot(4,1,2)
plot(filteredDataStatistics(:,6), 'linewidth', 2, 'color', blue)
hold on; grid on
ylabel('$y$', 'fontsize', 14, 'Interpreter', 'latex')
plot(filteredDataStatistics(:,11),'r', 'linewidth', 2, 'color', red)
plot(-1*filteredDataStatistics(:,11),'r', 'linewidth', 2, 'color', red)
axis([0 length(filteredDataStatistics) -500 500]);

subplot(4,1,3)
plot(filteredDataStatistics(:,3), 'linewidth', 2, 'color', blue)
hold on; grid on
ylabel('$\theta$', 'fontsize', 14, 'Interpreter', 'latex')
plot(filteredDataStatistics(:,8),'r', 'linewidth', 2, 'color', red)
plot(-1*filteredDataStatistics(:,8),'r', 'linewidth', 2, 'color', red)
axis([0 length(filteredDataStatistics) -2 2]);

subplot(4,1,4)
plot(filteredDataStatistics(:,4), 'linewidth', 2, 'color', blue)
hold on; grid on
ylabel('$\psi$', 'fontsize', 14, 'Interpreter', 'latex')
plot(filteredDataStatistics(:,9),'r', 'linewidth', 2, 'color', red)
plot(-1*filteredDataStatistics(:,9),'r', 'linewidth', 2, 'color', red)
axis([0 length(filteredDataStatistics) -10 10]);