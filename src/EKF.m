classdef EKF < handle
    
    properties
        H;              % measurement model
        S;              % innovation covariance
        v;              % innovation
        X;              % state vector
        z_hat;          % predicted measurement
        P;              % state covariance
        XPred;          % predicted state
        PPred;          % predicted state covariance
        Q;              % input noise covariance
        R;              % measurement noise covariance
        K;              % Kalman (filter) gain
        g;              % gravity
        Quaternion = [1 0 0 0]; % quaternion describing the Earth relative to the sensor
        Beta = 2;               % Madgwick's algorithm gain
    end
    
    methods
        function obj = EKF(system, init)
            % ekf Construct an instance of this class
            %
            %   Inputs:
            %       system          - system and noise models
            %       init            - initial state mean and covariance
            
            obj.H = system.H;
            obj.Q = system.Q;
            obj.R = system.R;
            obj.X = init.X;
            obj.P = init.P;
            obj.g = [0; 0; 9.81];
        end
        
        function [Rot, vel, pos] = separate_state(~, X)
            % Separate state vector into components
            O = X(1:3);               % Euler Angles
            Rot = eul2rotm(O', 'XYZ'); % Rot
            vel = X(4:6);             % Base Velocity
            pos = X(7:9);             % Base Position
        end
        
        function prediction(obj, w, a, dt)
            % w     - angular rate
            % a     - acceleration in Body Frame
            % dt    - sample time

            [Rot, vel, pos] = obj.separate_state(obj.X);
            q2 = rotm2quat(Rot);
            q1 = quaternion(q2);
            % propagate the state               
            q = obj.Quaternion; % short name local variable for readability
            aW = rotatepoint(q1, a')' - obj.g;     
            
            % Normalize accelerometer measurement
            a = a / norm(a);
            % Gradient decent algorithm corrective step
            F = [2*(q(2)*q(4) - q(1)*q(3)) - a(1)
                    2*(q(1)*q(2) + q(3)*q(4)) - a(2)
                    2*(0.5 - q(2)^2 - q(3)^2) - a(3)];
            J = [-2*q(3),  2*q(4), -2*q(1),	2*q(2)
                    2*q(2),  2*q(1),  2*q(4),	2*q(3)
                        0, -4*q(2),  -4*q(3),	0];
            step = (J'*F);
            step = step / norm(step); % normalize step magnitude
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 w(1) w(2) w(3)]) - obj.Beta * step';
            % Integrate to yield quaternion
            q = q + qDot * dt;
            obj.Quaternion = q / norm(q); % normalize quaternion
            
            % Now convert quaternion back to rotation matrix
            RPred = quatern2rotMat(quaternConj(obj.Quaternion));
            pPred = eye(3)*pos + dt*eye(3)*vel + ((dt^2)/(2))*eye(3)*aW;
            vPred = eye(3)*vel + dt*eye(3)*aW;

            OPred = rotm2eul(RPred, 'XYZ')';
            obj.XPred = [OPred;vPred;pPred];
            
            A = [0*eye(3)  0*eye(3) 0*eye(3);...
                 0*eye(3)  1*eye(3) 0*eye(3);...
                 0*eye(3) dt*eye(3) 1*eye(3)];
            obj.PPred = A * obj.P * A' + obj.Q;

        end
        
        function update(obj, z)
            % EKF update step
            %
            %   Inputs:
            %       z          - measurement

            % compute innovation statistics
            obj.v = z - obj.H * obj.XPred;
            obj.S = obj.H * obj.PPred * obj.H' + obj.R;
            
            % filter gain
            obj.K = obj.PPred * obj.H' * (obj.S \ eye(size(obj.S)));
            
            % correct the predicted state statistics
            obj.X = obj.XPred + obj.K * obj.v;
            I = eye(length(obj.X));
            obj.P = (I - obj.K * obj.H) * obj.PPred * (I - obj.K * obj.H)' ...
                    + obj.K * obj.R * obj.K';   % Joseph update form
        end
    end
end