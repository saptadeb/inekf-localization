% Converts a quaternion to its conjugate.
%
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-18

function qConj = quaternConj(q)
    qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
end