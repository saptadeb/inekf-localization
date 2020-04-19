clear; clc

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

% Run PF (TODO: need to add filter statistics)
% [filteredData, filteredDataStatistics] = PF();
[filteredData] = run_pf();

figure(1); hold on
plot(filteredData(:,5), filteredData(:,6), 'color', red)
plot(xGpsCg, yGpsCg, 'color', green)
plot(filteredData(:,11), filteredData(:,12), 'color', blue)
axis equal
title('\textbf{Robot location}', 'Interpreter', 'latex');
legend({'LIEKF', 'Consumer grade GPS', 'Ground truth'}, 'Location','Best', 'Interpreter', 'latex')
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')