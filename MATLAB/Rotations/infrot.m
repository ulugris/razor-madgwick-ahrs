function A = infrot(t)
% Infinitesimal rotation matrix

A = [1 -t(3) t(2); t(3) 1 -t(1); -t(2) t(1) 1];
