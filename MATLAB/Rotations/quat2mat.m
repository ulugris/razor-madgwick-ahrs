function A = quat2mat(q, unit)
% Get rotation matrix from quaternion

q22 = q(2)^2;
q33 = q(3)^2;
q44 = q(4)^2;

q12 = 2*q(1)*q(2);
q13 = 2*q(1)*q(3);
q14 = 2*q(1)*q(4);
q23 = 2*q(2)*q(3);
q24 = 2*q(2)*q(4);
q34 = 2*q(3)*q(4);

if nargin > 1 && unit
    A = diag(1 - 2*[q33 + q44, q44 + q22, q22 + q33]);
else
    q11 = q(1)^2;
    A = diag([q11 + q22 - q33 - q44, q11 - q22 + q33 - q44, q11 - q22 - q33 + q44]);
end

A(1, 2) = q23 - q14;
A(1, 3) = q24 + q13;
A(2, 1) = q23 + q14;
A(2, 3) = q34 - q12;
A(3, 1) = q24 - q13;
A(3, 2) = q34 + q12;
