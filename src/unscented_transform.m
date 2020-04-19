% Unscented transform class for uncertinty propagation through nonlinear models
% Refer to Sigmapoint Transformation, Chapter 4, State Estimation for Robotics, Barfoot
% Modified from Manni's code for EECS568
%
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-15

classdef unscented_transform < handle
    properties
        x;            % Input mean
        sigmaX;       % Input covariance
        L;            % Scaled Cholesky factor of sigmaX
        func;         % Nonlinear model
        y;            % Output mean
        sigmaY;       % Output covariance
        sigmaXY;      % Input-output cross covariance
        kappa;        % User-defined parameter to control the sigma points
        n;            % Input dimention
        sigmaPointsX; % 2n + 1 sigma points
        sigmaPointsY; % Mapped sigma points
        w;            % 2n + 1 sigma points weights
    end
    
    methods
        function obj = unscented_transform(inputMean, inputCov, f, kappa)
            obj.x = inputMean;
            obj.sigmaX = inputCov;
            obj.func = f;
            obj.n = numel(inputMean);
            obj.kappa = kappa;
        end
            
        function sigma_points(obj)
            % Sigma points around the reference point
            obj.L = sqrt(obj.n + obj.kappa) * chol(obj.sigmaX, 'lower');
            xs = obj.x(:, ones(1, numel(obj.x)));
            obj.sigmaPointsX = [obj.x, xs + obj.L, xs - obj.L];
            obj.w = zeros(2 * obj.n + 1, 1);
            obj.w(1) = obj.kappa / (obj.n + obj.kappa);
            obj.w(2:end) = 1 / (2 * (obj.n + obj.kappa));
        end
        
        function propagate(obj)
            % Propagate the input Gaussian using unscented transform
            obj.sigma_points();
            obj.y = 0;
            obj.sigmaY = 0;
            obj.sigmaPointsY = [];
            for j = 1: 2*obj.n + 1
                obj.sigmaPointsY(:,j) = obj.func(obj.sigmaPointsX(:,j));
                obj.y = obj.y + obj.w(j) * obj.sigmaPointsY(:,j);
            end
            diff = (obj.sigmaPointsY - obj.y);
            obj.sigmaY = diff * diag(obj.w) * diff';
            obj.sigmaXY = (obj.sigmaPointsX - obj.x) * diag(obj.w) * diff';
        end
    end
end