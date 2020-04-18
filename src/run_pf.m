%% Particle filter


function varargout = run_pf()
%% Load data
addpath([cd, '/../data'])
data = load('data_2013-01-10.mat');

%% System
% Position  | States    | Description
% ------------------------------------
% 1         | x_k       | position x      
% 2         | y_k       | position y
% 3         | z_k       | position z
% 4         | xdot_k    | velocity x
% 5         | ydot_k    | velocity y
% 6         | zdot_k    | velocity z
% ------------------------------------
% 7         | phi_k     | roll
% 8         | theta_k   | pitch
% 9         | psi_k     | yaw
% ------------------------------------
%{
sys = [];

% convert data to quaternions
cr = @(e) cos(e.roll * 0.5);
sr = @(e) sin(e.roll * 0.5);
cp = @(e) cos(e.pitch * 0.5);
sp = @(e) sin(e.pitch * 0.5);
cy = @(e) cos(e.yaw * 0.5);
sy = @(e) sin(e.yaw * 0.5);

qw = @(e) cr(e) * cp(e) * cy(e) + sr(e) * sp(e) * sy(e);
qx = @(e) sr(e) * cp(e) * cy(e) - cr(e) * sp(e) * sy(e);
qy = @(e) cr(e) * sp(e) * cy(e) + sr(e) * cp(e) * sy(e);
qz = @(e) cr(e) * cp(e) * sy(e) - sr(e) * sp(e) * cy(e);

% quaternion orientation of robot
sys.q = @(e) quaternion(qw(e), qx(e), qy(e), qz(e));

% convert accelaration to world frame
sys.accW = @(accB, q) q * accB * q';

% robot motion model
% Inputs:
%   x       - state x
%   accb    - acceleration in body frame
%   e       - orientation in euler angles
%   dt      - sample time
sys.f = @(x, accB, dt) [eye(3) eye(3)*dt; 0*eye(3) eye(3)]*x(1:6) + ...
    [(dt^2/2)*eye(3); dt*eye(3)] * accB;...sys.accW(accB, sys.q(e));

%}
% measurement model
sys.h = @(x) [eye(3), 0*eye(3), 0*eye(3)] * x;

% process noise covariance
sys.Q = 1e1 * eye(9);

% measurement noise covariance
sys.R = 1e1 * eye(3);

% initialization for filter
init.n = 100;                           % number of particles
init.Sigma = 1e1 * eye(9);              % initial sigma

%% Initialization
% Use first GPS measurement as initial position 
[x, y, z] = latlngalt2xyz(data.gps_cg.latitude(1), ...
                          data.gps_cg.longitude(1), ...
                          data.gps_cg.altitude(1));
gpsFirstTimestamp = data.gps_cg.timestamp(1);
imuFirstIndex = find(data.imu.timestamp > gpsFirstTimestamp, 1);

filter = PF(init);

gpsIndex = 2;
lastValidAltitude = data.gps_cg.altitude(1);
filteredData = zeros(length(data.imu.accel_x)-imuFirstIndex, 3);

for imuIndex = imuFirstIndex: length(data.imu.accel_x)-1
    acceleration = [data.imu.accel_x(imuIndex);
                    data.imu.accel_y(imuIndex);
                    data.imu.accel_z(imuIndex)];
    angularRate = [data.imu.gyro_x(imuIndex);
                   data.imu.gyro_y(imuIndex);
                   data.imu.gyro_z(imuIndex)];
    imuTimestamp = data.imu.timestamp(imuIndex);
    nextImuTimeStamp = data.imu.timestamp(imuIndex+1);
    dt = nextImuTimeStamp - imuTimestamp;
    
    % If there is GPS data coming between two IMU timestamps
    if gpsIndex <= length(data.gps_cg.timestamp) && nextImuTimeStamp > data.gps_cg.timestamp(gpsIndex)
        % In case gps_cg.altitude is NaN
        if isnan(data.gps_cg.altitude(gpsIndex))
            altitude = lastValidAltitude;
        else
            altitude = data.gps_cg.altitude(gpsIndex);
            lastValidAltitude = altitude;
        end
        [x, y, z] = latlngalt2xyz(data.gps_cg.latitude(gpsIndex), ...
                                  data.gps_cg.longitude(gpsIndex), ...
                                  altitude);
        Y = [x;y;z];
        filter.update(Y);
        gpsIndex = gpsIndex + 1;
    end
    
    filter.prediction(angularRate, acceleration, dt);
    filteredData(imuIndex - imuFirstIndex + 1,:) = filter.X(1:3,5)';
end


if nargout >= 1
    varargout{1} = sys;
    varargout{2} = data;
end
end