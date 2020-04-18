classdef PF < handle
    % Particle filter class for state estimation of a nonlinear system
    % The implementation follows the Sample Importance Resampling (SIR)
    % filter a.k.a bootstrap filter.
    
    properties
        f;              % process model
        h;              % measurement model
        x;              % state vector
        Sigma;          % state covariance
        Q;              % input noise covariance
        LQ;             % Cholesky factor of Q
        R;              % measurement noise covariance
        p;              % particles
        n;              % number of particles
%         dt;             % sampling time
        Neff;           % effective number of particles
    end
    
    methods
        function obj = PF(system, init)
            % particle_filter construct an instance of this class
            %
            %   Inputs:
            %       system          - system and noise models
            %       init            - initialization parameters
            obj.f = system.f;
            obj.Q = system.Q;
            obj.LQ = chol(obj.Q, 'lower');
            obj.h = system.h;
            obj.R = system.R;
            obj.n = init.n;
            
            % initialize particles
            obj.p = [];
            wu = 1/obj.n; % uniform weights
            L_init = chol(init.Sigma, 'lower');
            for i = 1:obj.n
                obj.p.x(:,i) = L_init * randn(size(init.x,1),1) + init.x;
                obj.p.w(i,1) = wu;
            end
        end
        
        function [A] = skew(~, v)
            % Convert from vector to skew symmetric matrix
            A = [    0, -v(3),  v(2);
                  v(3),     0, -v(1);
                 -v(2),  v(1),    0];
        end
        
        function prediction(obj)
            % motion model
            for i = 1:obj.n
                % propagate the particles!
                [R, v, p] = obj.separate_state(obj.X);
            
                % Pose dynamics
                RPred = R * expm(obj.skew(w * dt));
                vPred = v + (R * a + obj.g) * dt;
                pPred = p + v * dt + 0.5 * (R * a + obj.g) * dt^2;
                
                obj.p.x(:,i) = obj.f(obj.p.x(:,i));
            end
        end
        
        function update(obj, z)
            % compute important weight for each particle based on the 
            % obtained range and bearing measurements
            %
            %   Inputs:
            %       z          - (lat, lon, alt)
            w = zeros(obj.n,1); % importance weights
            for i = 1:obj.n
                 % compute innovation statistics
                 % We know here z(2) is an angle
                 v = z - obj.h(obj.p.x(:,i));
                 w(i) = mvnpdf(v, [0;0;0], obj.R);
            end
            % update and normalize weights
            obj.p.w = obj.p.w .* w; % since we used motion model to sample
            obj.p.w = obj.p.w ./ sum(obj.p.w);
            % compute effective number of particles
            obj.Neff = 1 / sum(obj.p.w.^2);
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
    end
end