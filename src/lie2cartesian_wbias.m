function lie2cartesian_wbias(filter)
    f = @func;
    kappa = 2;
%     state = [filter.X(1:3, 1:5); zeros(2, 3), eye(2)];
    se2 = logm(filter.X);
    inputMean = [se2(3,2); se2(1,3); se2(2,1); se2(1:3, 4); se2(1:3, 5)];
    ut = unscented_transform(inputMean, filter.P(1:9, 1:9), f, kappa);
    ut.propagate();
    filter.XCart = state2cartesian(filter.X);
    filter.PCart = ut.sigmaY;
end

function y = func(x)
    Lg = zeros(5);
    Lg(1:3, :) = [skew(x(1:3)), x(4:6), x(7:9)];
    SE3 = expm(Lg);
    
    y = state2cartesian(SE3);
end

function A = skew(v)
    % Convert from vector to skew symmetric matrix
    A = [    0, -v(3),  v(2);
          v(3),     0, -v(1);
         -v(2),  v(1),    0];
end

function x = state2cartesian(X)
    % Convert state matrix (5x5) to roll, pitch, yaw, px, py, pz, vx, vy, vz
    x = [rotm2eul(X(1:3, 1:3), 'XYZ')';
         X(1:3, 5);
         X(1:3, 4)];
end