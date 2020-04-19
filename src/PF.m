classdef PF < handle
    % Particle filter class for state estimation of a nonlinear system
    % The implementation follows the Sample Importance Resampling (SIR)
    % filter a.k.a bootstrap filter.
    
    properties
        h;              % measurement model
        X;              % state vector
        Sigma;          % state covariance
        Q;              % input noise covariance
        LQ;             % Cholesky factor of Q
        R;              % measurement noise covariance
        p;              % particles
        n;              % number of particles
        XCart;
        g;
        Neff;           % effective number of particles
        Quaternion = [1 0 0 0]; % quaternion describing the Earth relative to the sensor
        Beta = 2;               % Madgwick's algorithm gain
    end
    
    methods
        function obj = PF(system, init)
            % particle_filter construct an instance of this class
            %
            %   Inputs:
            %       system          - system and noise models
            %       init            - initialization parameters
            obj.Q = system.Q;
            obj.LQ = chol(obj.Q, 'lower');
            obj.h = system.h;
            obj.R = system.R;
            obj.n = init.n;
            obj.XCart = zeros(6,1);
            obj.g = [0; 0; 9.81];
            % initialize particles
            obj.p = [];
            wu = 1/obj.n; % uniform weights
            L_init = chol(init.Sigma, 'lower');
            for i = 1:obj.n
                obj.p.x(:,i) = L_init * randn(size(init.X,1),1) + init.X;
                obj.p.w(i,1) = wu;
            end
        end
        
        function [Rot, vel, pos] = separate_state(~, X)
            % Separate state vector into components
            O = X(1:3);             % Euler Angles
            Rot = eul2rotm(O', 'XYZ');% Rotation matrix
            vel = X(4:6);             % Base Velocity
            pos = X(7:9);             % Base Position
        end
        
        function X = construct_state(~, Rot, vel, pos)
            % Construct matrix from separate states
            O = rotm2eul(Rot, 'XYZ')';
            X = [O; vel; pos];
        end
        

        function prediction(obj, w, a, dt)
            % motion model
            for i = 1:obj.n
                [Rot, vel, pos] = obj.separate_state(obj.p.x(:, i));
                
                % propagate the particles!               
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
                
                obj.p.x(:,i) = obj.construct_state(RPred, vPred, pPred);
                
            end
        end
        
        function update(obj, z)
            % compute important weight for each particle based on the 
            % obtained range and bearing measurements
            %
            %   Inputs:
            %       z          - (xyz from lat, lon, alt)
            w = zeros(obj.n,1); % importance weights
            for i = 1:obj.n
                 % compute innovation statistics
                 v = z - obj.h(obj.p.x(:,i));
                 w(i) = mvnpdf(v, [0;0;0], obj.R);
            end
            % update and normalize weights
            obj.p.w = obj.p.w .* w; % since we used motion model to sample
            obj.p.w = obj.p.w ./ sum(obj.p.w);
            % compute effective number of particles
            obj.Neff = 1 / sum(obj.p.w.^2);

            if obj.Neff < obj.n/5
                obj.resampling();
            end
            wtot = sum(obj.p.w);
            if wtot > 0
                obj.XCart(1) = sum(obj.p.x(1,:)' .* obj.p.w) / wtot;
                obj.XCart(2) = sum(obj.p.x(2,:)' .* obj.p.w) / wtot;
                obj.XCart(3) = sum(obj.p.x(3,:)' .* obj.p.w) / wtot;
                obj.XCart(4) = sum(obj.p.x(7,:)' .* obj.p.w) / wtot;
                obj.XCart(5) = sum(obj.p.x(8,:)' .* obj.p.w) / wtot;
                obj.XCart(6) = sum(obj.p.x(9,:)' .* obj.p.w) / wtot;
            else
%                 warning('Total weight is zero or nan!')
%                 disp(wtot)
                obj.XCart = nan(6,1);
            end

        end
        
        function resampling(obj)
            % low variance resampling
            W = cumsum(obj.p.w);          
            r = rand / obj.n ;
            j = 1;
            for i = 1:obj.n
                u = r + (i-1) / obj.n;
                while u > W(j)
                    j = j + 1;
                end
                obj.p.x(:,i) = obj.p.x(:,j);
                obj.p.w(i) = 1/obj.n;
            end
        end
%{
        function meanAndVariance(obj)
            obj.mu = mean(obj.particles, 2); 
            % orientation is a bit more tricky.
            sinSum = 0;
            cosSum = 0;
            for s = 1:obj.n
                cosSum = cosSum + cos(obj.particles(3,s));
                sinSum = sinSum + sin(obj.particles(3,s));
            end
            obj.mu(3) = atan2(sinSum, cosSum);     
            % Compute covariance.
            zeroMean = obj.particles - repmat(obj.mu, 1, obj.n);
            for s = 1:obj.n
                zeroMean(3,s) = wrapTo2Pi(zeroMean(3,s));
            end
            obj.Sigma = zeroMean * zeroMean' / obj.n;
        end
%}
    end
end