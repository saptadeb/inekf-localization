% Convert GPS latitude, longitude, altitude measurements to local coordinate frame x, y, z
% Refer to equation (1) (2), page 7, University of Michigan North Campus Long-Term Vision and Lidar Datasete.pdf 
% 
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-14

function [x,y,z] = latlngalt2xyz(lat, lng, alt)
    lat0 = deg2rad(42.293227);
    lng0 = deg2rad(-83.709657);
    alt0 = 270.;
    re = 6378135.;
    rp = 6356750.;
    d = (re * cos(lat0))^2 + (rp * sin(lat0))^2;
    rns = (re*rp)^2 / d^(3/2);
    rew = re^2 / sqrt(d);
    x = sin(lat - lat0) * rns;
    y = sin(lng - lng0) * rew * cos(lat0);
    z = alt0 - alt;
end