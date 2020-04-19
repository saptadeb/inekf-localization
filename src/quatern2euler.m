% Converts a quaternion orientation to ZYX Euler angles
% phi is a rotation around X, theta around Y and psi around Z.
%
% Author: Hao Zhou <zhh@umich.edu>
% Date:   2020-04-18

function euler = quatern2euler(q)
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;

    phi = atan2(R(3,2,:), R(3,3,:));
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2));
    psi = atan2(R(2,1,:), R(1,1,:));

    euler = [phi(1,:)' theta(1,:)' psi(1,:)'];
end

