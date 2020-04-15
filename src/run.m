% LIEKF test run file
%
% Author: Hao Zhou <zhh@umich.edu>
% Date: 2020-04-14

function varargout = run()
%% Load data
addpath([cd, '/../data'])
data = load('data_2013-01-10.mat');

%% Initialization
% Use first GPS measurement as initial position 
[x, y, z] = latlngalt2xyz(data.gps_cg.latitude(1), ...
                          data.gps_cg.longitude(1), ...
                          data.gps_cg.altitude(1));
gpsFirstTimestamp = data.gps_cg.timestamp(1);
imuFirstIndex = find(data.imu.timestamp > gpsFirstTimestamp, 1);

init.X = [eye(3), zeros(3,1), [x;y;z];
          zeros(2,3), eye(2)];
init.P = eye(9);
init.Qg = 0.01 * eye(3);
init.Qa = 0.1 * eye(3);
init.Qp = blkdiag(2*eye(3), zeros(2));
filter = LIEKF(init);

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
        Y = [x;y;z;0;1];
        filter.update(Y);
        gpsIndex = gpsIndex + 1;
    end
    
    filter.prediction(angularRate, acceleration, dt);
    filteredData(imuIndex - imuFirstIndex + 1,:) = filter.X(1:3,5)';
end

if nargout >= 1
    varargout{1} = filteredData;
end