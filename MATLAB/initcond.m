function [x, P] = initcond(a, m)

d = a'/norm(a);
e = cross(d, m');
e = e/norm(e);
n = cross(e, d);

A = [n e d]';

x = [mat2quat(A) zeros(1, 3)];
P = diag([0.17 0.07 0.08 0.05 0 0 0]);
