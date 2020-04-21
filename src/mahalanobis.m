% Compute the mahalanobis distance between the filtered data and ground-truth data
% 
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-16

function result = mahalanobis(filter,groundtruth,type)
    if (type == 1)  %% for LIEKF
        diff = groundtruth(1:5) - filter.XCart(1:5);
        result(1) = diff' * (filter.PCart(1:5, 1:5) \ diff);
        result(2:6) = diff;
        result(7:11) = 3 * sqrt(diag(filter.PCart(1:5,1:5)))';
    else            %% for EKF
        x = [filter.X(1:3); filter.X(7:8)];
        p = [filter.P(1:3,1:3), filter.P(1:3,7:8);...
             filter.P(7:8,1:3), filter.P(7:8,7:8)];        
        diff = groundtruth(1:5) - x;
        result(1) = diff' * (p \ diff);
        result(2:6) = diff;
        result(7:11) = 3 * sqrt(diag(p))';
    end    
end