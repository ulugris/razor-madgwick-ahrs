function pq = quatmult(p, q)
% Quaternion product

pq(1) = p(1)*q(1) - p(2)*q(2) - p(3)*q(3) - p(4)*q(4);
pq(2) = p(1)*q(2) + p(2)*q(1) + p(3)*q(4) - p(4)*q(3);
pq(3) = p(1)*q(3) + p(3)*q(1) - p(2)*q(4) + p(4)*q(2);
pq(4) = p(1)*q(4) + p(4)*q(1) + p(2)*q(3) - p(3)*q(2);
