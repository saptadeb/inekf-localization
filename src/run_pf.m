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
sys.f = @(x, accB, e, dt) [eye(3) eye(3)*dt; 0*eye(3) eye(3)]*x(1:6) + ...
    [(dt^2/2)*eye(3); dt*eye(3)] * sys.accW(accB, sys.q(e));

% measurement model
sys.h = @(x) [eye(3), 0*eye(3), 0*eye(3)] * x;

% 
sys.Q = 1e1 * eye(2);

% 
sys.R = 1;

%{
%% Initialization
% Use first GPS measurement as initial position 
[x, y, z] = latlngalt2xyz(data.gps_cg.latitude(1), ...
                          data.gps_cg.longitude(1), ...
                          data.gps_cg.altitude(1));
gpsFirstTimestamp = data.gps_cg.timestamp(1);
imuFirstIndex = find(data.imu.timestamp > gpsFirstTimestamp, 1);


%}

if nargout >= 1
    varargout{1} = sys;
    varargout{2} = data;
end
end