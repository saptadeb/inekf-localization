classdef ekf < handle
    
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
        function obj = ekf(system, init)
            % ekf Construct an instance of this class
            %
            %   Inputs:
            %       system          - system and noise models
            %       init            - initial state mean and covariance
            
            obj.H = system.H;
            obj.Q = system.Q;
            obj.R = system.R;
            obj.x = init.x;
            obj.P = init.P;
            obj.g = [0; 0; 9.81];
        end
        
        function [O, vel, pos] = separate_state(~, X)
            % Separate state vector into components
            O = X(1:3);               % Euler Angles
            Rot = eul2rotm(O, 'XYZ'); % Rot
            vel = X(4:6);             % Base Velocity
            pos = X(7:9);             % Base Position
        end
        
        function X = construct_state(~, Rot, vel, pos)
            % Construct matrix from separate states
            O = rotm2eul(Rot, 'XYZ')';
            X = [O; vel; pos];
        end


        function prediction(obj, w, a, dt)
            % w     - angular rate
            % a     - acceleration in Body Frame
            % dt    - sample time

            [Rot, vel, pos] = obj.separate_state(obj.X);
            q1 = rotm2quat(Rot);

            % propagate the state               
            q = obj.Quaternion; % short name local variable for readability

            aW = q1 * a * q1' - obj.g;      

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
            % q = q + qDot * obj.SamplePeriod;
            q = q + qDot * dt;
            obj.Quaternion = q / norm(q); % normalize quaternion
            
            % Now convert quaternion back to rotation matrix
            RPred = quatern2rotMat(quaternConj(obj.Quaternion));

            [pPred; vPred] = [eye(3), dt*eye(3) ; zeros(3,3), eye(3)] * [pos; vel] +...
                [((dt^2)/(2))*eye(3) ; dt*eye(3)] * aW;

            obj.XPred = construct_state(RPred, vPred, pPred);
            
            A = [0*eye(3)  0*eye(3) 0*eye(3);...
                 0*eye(3)  1*eye(3) 0*eye(3);...
                 0*eye(3) dt*eye(3) 1*eye(3)];
            obj.PPred = A * obj.P * A' + obj.Q;

        end
%{
            %Evaluate G with mean and input
            G = obj.Gfun(obj.x,u);

            [Rot, vel, pos] = obj.separate_state(obj.x);
                
            % propagate the state               
            q = obj.Quaternion; % short name local variable for readability
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
            % q = q + qDot * obj.SamplePeriod;
            q = q + qDot * dt;
            obj.Quaternion = q / norm(q); % normalize quaternion
            
            % Now convert quaternion back to rotation matrix
            RPred = quatern2rotMat(quaternConj(obj.Quaternion));
            vPred = vel + (Rot * a + obj.g) * dt;
            pPred = pos + vel * dt + 0.5 * (Rot * a + obj.g) * dt^2;

            obj.x = obj.construct_state(RPred, vPred, pPred);
            
            obj.Sigma_pred = G*obj.Sigma*G'+V*obj.M(u)*V';
%}

        
        function update(obj, z)
            % EKF update step
            %
            %   Inputs:
            %       z          - measurement

            % compute innovation statistics
            obj.v = z - obj.H * obj.X;
            obj.S = obj.H * obj.PPred * obj.H' + obj.R;
            
            % filter gain
            obj.K = obj.PPred * obj.H' * (obj.S \ eye(size(obj.S)));
            
            % correct the predicted state statistics
            obj.X = obj.XPred + obj.K * obj.v;
            I = eye(length(obj.X));
            obj.P = ...
                (I - obj.K * obj.H) * obj.PPred * (I - obj.K * obj.H)' ...
                    + obj.K * obj.R * obj.K'; % Joseph update form
        end
    end
end