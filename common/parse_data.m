function data = parse_data(file, type)

% read data out as a table.
% Note that this actually reads out data as 'single' instead
% of 'double'. Could change to reading out each line to improve
% precision. 
rawData = table2array(readtable(file));

switch (type)
    case 'imu'
        % File format is as follows:
        % | TimeStamp | roll | pitch | yaw |
        data.timeStamps = rawData(:, 1);
        data.e.roll = rawData(:, 2);
        data.e.pitch = rawData(:, 3);
        data.e.yaw = rawData(:, 4);
        
        % convert data to quaternions
        cr = cos(data.e.roll .* 0.5);
        sr = sin(data.e.roll .* 0.5);
        cy = cos(data.e.yaw .* 0.5);
        sy = sin(data.e.yaw .* 0.5);
        cp = cos(data.e.pitch .* 0.5);
        sp = sin(data.e.pitch .* 0.5);

        data.q.w = cr .* cp .* cy + sr .* sp .* sy;
        data.q.x = sr .* cp .* cy - cr .* sp .* sy;
        data.q.y = cr .* sp .* cy + sr .* cp .* sy;
        data.q.z = cr .* cp .* sy - sr .* sp .* cy;
        
    otherwise
        error('unknown type received');
end

end
