function qv = quatvmult(q, v)
% Quaternion-vector product

qv(1) =-q(2)*v(1) - q(3)*v(2) - q(4)*v(3);
qv(2) = q(1)*v(1) + q(3)*v(3) - q(4)*v(2);
qv(3) = q(1)*v(2) - q(2)*v(3) + q(4)*v(1);
qv(4) = q(1)*v(3) + q(2)*v(2) - q(3)*v(1);
