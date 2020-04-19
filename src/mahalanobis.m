% Compute the mahalanobis distance between the filtered data and ground-truth data
% 
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-16

function result = mahalanobis(filter,groundtruth)
    diff = groundtruth(1:5) - filter.XCart(1:5);
    result(1) = diff' * (filter.PCart(1:5, 1:5) \ diff);
    result(2:6) = diff;
    result(7:11) = 3 * sqrt(diag(filter.PCart(1:5,1:5)))';
end