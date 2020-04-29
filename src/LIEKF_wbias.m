% Left-Invariant Extended Kalman Filter for IMU and GPS fusing
% with IMU bias included

classdef LIEKF_wbias < handle
    properties 
        X;         % State (R, v, p) mean
        XPred;     % State mean after prediction step
        theta;     % Bias (bg, ba) mean
        thetaPred; % Bias mean after prediction step
        P;         % Augmented state (R, v, p, bg, ba) covariance matrix
        PPred;     % Augmented state covariance matrix after prediction step
        
        Qg;        % Gyro covariance matrix
        Qa;        % Accel covariance matrix
        Qp;        % GPS position covariance matrix
        Qbg;       % Additive Gyro drift noise covariance matrix
        Qba;       % Additive Accel drift noise covariance matrix
        
        H;         % H matrix in update step
        b;         % b matrix in update step
        
        g;         % Gravity vector
        
        XCart;     % State mean in cartesian frame: (r,p,y,px,py,pz,vx,vy,vz)
        PCart;     % State covariance matrix in cartesian frame
        
        Quaternion = [1 0 0 0]; % Quaternion describing the Earth relative to the sensor
        Beta = 1;               % Madgwick's algorithm gain
    end
    
    methods
        function obj = LIEKF_wbias(init)
            obj.X = init.X;
            obj.theta = init.theta;
            obj.P = init.P;
            obj.Qg = init.Qg;
            obj.Qa = init.Qa;
            obj.Qp = init.Qp;
            obj.Qbg = init.Qbg;
            obj.Qba = init.Qba;
            obj.H = [zeros(3), zeros(3), eye(3), zeros(3), zeros(3)];
            obj.b = [zeros(4,1); 1];
            obj.g = [0; 0; 9.81];
        end
        
        function [R, v, p, bg, ba] = separate_state(~, X, theta)
            % Separate state vector into components
            R = X(1:3, 1:3); % Orientation
            v = X(1:3, 4);   % Base Velocity
            p = X(1:3, 5);   % Base Position
            bg = theta(1:3); % Gyroscope Bias
            ba = theta(4:6); % Accelerometer Bias
        end
        
        function [X, theta] = construct_state(~, R, v, p, bg, ba)
            % Construct matrix from separate states
            X = eye(5);
            X(1:3,1:5) = [R, v, p];
            theta = [bg; ba];
        end
        
        function A = skew(~, v)
            % Convert from vector to skew symmetric matrix
            A = [    0, -v(3),  v(2);
                  v(3),     0, -v(1);
                 -v(2),  v(1),    0];
        end
        
        function [dX, dtheta] = exp(obj, v)
            % Exponential map of SE_2(3)
            Lg = zeros(5);
            Lg(1:3, :) = [obj.skew(v(1:3)), v(4:6), v(7:9)];
        	dX = expm(Lg);
            dtheta = v(10:15);
        end
        
        function prediction(obj, w, a, dt)
            % Left-Invariant Extended Kalman Filter prediction step
            [R, v, p, bg, ba] = obj.separate_state(obj.X, obj.theta);
            
            % Bias corrected IMU information
            wBar = w - bg;
            aBar = a - ba;
            
            % Pose dynamics
            RPred = R * expm(obj.skew(wBar * dt));
%             % Instead of the above simple equation to update attitude, we use Madgwick's algorithm
%             q = obj.Quaternion; % short name local variable for readability
%             % Normalize accelerometer measurement
%             aBarNormalized = aBar / norm(aBar);
%             % Gradient decent algorithm corrective step
%             F = [2*(q(2)*q(4) - q(1)*q(3)) - aBarNormalized(1)
%                  2*(q(1)*q(2) + q(3)*q(4)) - aBarNormalized(2)
%                  2*(0.5 - q(2)^2 - q(3)^2) - aBarNormalized(3)];
%             J = [-2*q(3),  2*q(4), -2*q(1),	2*q(2)
%                   2*q(2),  2*q(1),  2*q(4),	2*q(3)
%                        0, -4*q(2),  -4*q(3),	0];
%             step = (J'*F);
%             step = step / norm(step); % normalize step magnitude
%             % Compute rate of change of quaternion
%             qDot = 0.5 * quaternProd(q, [0 wBar(1) wBar(2) wBar(3)]) - obj.Beta * step';
%             % Integrate to yield quaternion
%             % q = q + qDot * obj.SamplePeriod;
%             q = q + qDot * dt;
%             obj.Quaternion = q / norm(q); % normalize quaternion
%             % Now convert quaternion back to rotation matrix
%             RPred = quatern2rotMat(quaternConj(obj.Quaternion));
            
            vPred = v + (R * aBar + obj.g) * dt;
            pPred = p + v * dt + 0.5 * (R * aBar + obj.g) * dt^2;
            
            % Bias Dynamics
            bgPred = bg;
            baPred = ba;
            
            % Linearized continuous invariant error dynamics
            Ac = [-obj.skew(wBar),        zeros(3),        zeros(3),  -eye(3), zeros(3);
                  -obj.skew(aBar), -obj.skew(wBar),        zeros(3), zeros(3),  -eye(3);
                         zeros(3),          eye(3), -obj.skew(wBar), zeros(3), zeros(3);
                     zeros(6, 15)];
            % Discrete-time state transition matrix \Phi = expm(Ac * dt)
            Phik = expm(Ac * dt);
            % Phik = eye(size(Ac)) + Ac*dt;
            
            % Continous covariance matrix
            Qc = blkdiag(obj.Qg, obj.Qa, zeros(3), obj.Qbg, obj.Qba);
            % Discretized covariance matrix
            Qk = Phik * Qc * Phik' * dt;
            
            % Construct predicted state
            [obj.XPred, obj.thetaPred] = obj.construct_state(RPred, vPred, pPred, bgPred, baPred);

            % Predict Covariance
            obj.PPred = Phik * obj.P * Phik' + Qk;
        end
 
        function update(obj, Y)
            % Update state and covariance from a measurement
            % Compute Kalman gain L
            invX = obj.XPred \ eye(size(obj.XPred));
            N = invX * obj.Qp * invX';
            N = N(1:3, 1:3);
            S = obj.H * obj.PPred * obj.H' + N;
            L = (obj.PPred * obj.H') / S;
            
            % Update state
            innovation = invX * Y - obj.b;
            innovation = innovation(1:3);
            delta = L * innovation;
            [dX, dTheta] = obj.exp(delta);
            obj.X = obj.XPred * dX;
            obj.theta = obj.thetaPred + dTheta;
            
            % Update covariance
            I = eye(size(obj.PPred));
            % Joseph update form
            obj.P = (I - L * obj.H) * obj.PPred * (I - L * obj.H)' + L * N * L';
        end
    end % endmethods
end % classdef