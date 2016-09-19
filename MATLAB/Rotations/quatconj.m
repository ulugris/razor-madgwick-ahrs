function q = quatconj(q)
% Quaternion conjugate

q = [q(1) -q(2:4)];
