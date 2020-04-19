% Implementation of Madgwick's IMU and AHRS algorithms
% For more information see: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
%
% Author: SOH Madgwick
% Date:   2011-09-28
%
% Modifier: Hao Zhou <zhh@umich.edu>
% Date:     2020-04-18

classdef MadgwickAHRS < handle
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0]; % output quaternion describing the Earth relative to the sensor
        Beta = 1;               % algorithm gain
    end

    %% Public methods
    methods (Access = public)
        function obj = MadgwickAHRS(varargin)
            for i = 1:2:nargin
                if strcmp(varargin{i}, 'SamplePeriod')
                    obj.SamplePeriod = varargin{i+1};
                elseif strcmp(varargin{i}, 'Quaternion') 
                    obj.Quaternion = varargin{i+1};
                elseif strcmp(varargin{i}, 'Beta')
                    obj.Beta = varargin{i+1};
                else
                    error('Invalid argument');
                end
            end
        end
        
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer, dt)
            q = obj.Quaternion; % short name local variable for readability

            % Normalize accelerometer measurement
            if(norm(Accelerometer) == 0)
                return; 
            end	% handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer); % normalize magnitude

            % Gradient decent algorithm corrective step
            F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)];
            J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
                2*q(2),     2*q(1),     2*q(4),	2*q(3)
                0,         -4*q(2),    -4*q(3),	0    ];
            step = (J'*F);
            step = step / norm(step); % normalize step magnitude

            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * step';

            % Integrate to yield quaternion
            % q = q + qDot * obj.SamplePeriod;
            q = q + qDot * dt;
            obj.Quaternion = q / norm(q); % normalize quaternion
        end
        
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer, dt)
            q = obj.Quaternion; % short name local variable for readability

            % Normalize accelerometer measurement
            if (norm(Accelerometer) == 0)
                return; 
            end	% handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer); % normalize magnitude

            % Normalise magnetometer measurement
            if (norm(Magnetometer) == 0)
                return; 
            end	% handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer); % normalize magnitude

            % Reference direction of Earth's magnetic feild
            h = quaternProd(q, quaternProd([0 Magnetometer], quaternConj(q)));
            b = [0 norm([h(2) h(3)]) 0 h(4)];

            % Gradient decent algorithm corrective step
            F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)
                2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - Magnetometer(1)
                2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - Magnetometer(2)
                2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - Magnetometer(3)];
            J = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
                2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
                0,                         -4*q(2),                    -4*q(3),                         0
                -2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
                -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
                2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];
            step = (J'*F);
            step = step / norm(step); % normalize step magnitude

            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * step';

            % Integrate to yield quaternion
            % q = q + qDot * obj.SamplePeriod;
            q = q + qDot * dt;
            obj.Quaternion = q / norm(q); % normalize quaternion
        end
    end
end