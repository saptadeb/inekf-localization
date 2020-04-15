% Plot results generated from LIEKF
% 
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-15

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

% Run LIEKF
filteredData = run();

figure(1); hold on
plot(data.ground_truth.x, data.ground_truth.y, 'color', blue)
plot(xGpsCg, yGpsCg, 'color', green)
plot(filteredData(:,1), filteredData(:,2), 'color', red)
axis equal
legend({'Ground truth', 'Consumer grade GPS', 'LIEKF'}, 'Location','southwest', 'Interpreter', 'latex')
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')