function ypr = yawpitchroll(x)
% Get yaw-pitch-roll from rotation matrix or quaternion

if numel(x) == 9
    % Input is a rotation matrix
    ypr(1) = atan2(x(2), x(1));
    ypr(2) = -asin(x(3));
    ypr(3) = atan2(x(6), x(9));
else
    % Input is a quaternion
    ypr(1) = atan2(2*(x(2)*x(3) + x(1)*x(4)), x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2);
    ypr(2) = -asin(2*(x(2)*x(4) - x(1)*x(3)));
    ypr(3) = atan2(2*(x(3)*x(4) + x(1)*x(2)), x(1)^2 - x(2)^2 - x(3)^2 + x(4)^2);
end
