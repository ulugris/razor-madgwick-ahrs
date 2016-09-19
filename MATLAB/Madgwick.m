function q = MadgwickMod(q, a, m, g, dt, X)

beta = X(1);
zeta = X(2);

q11 = q(1)^2;
q22 = q(2)^2;
q33 = q(3)^2;
q44 = q(4)^2;

q12 = q(1)*q(2);
q13 = q(1)*q(3);
q14 = q(1)*q(4);
q23 = q(2)*q(3);
q24 = q(2)*q(4);
q34 = q(3)*q(4);

A1 = [0.5 * (q11 + q22 - q33 - q44); q23 - q14; q24 + q13];
A2 = [q23 + q14; 0.5 * (q11 - q22 + q33 - q44); q34 - q12];

F = [a*A1 a*A2 m*A2];

J11 = a(1)*q(1) - a(2)*q(4) + a(3)*q(3);
J12 = a(1)*q(2) + a(2)*q(3) + a(3)*q(4); 
J13 =-a(1)*q(3) + a(2)*q(2) + a(3)*q(1);
J14 =-a(1)*q(4) - a(2)*q(1) + a(3)*q(2);
J31 = m(1)*q(4) + m(2)*q(1) - m(3)*q(2);
J32 = m(1)*q(3) - m(2)*q(2) - m(3)*q(1);
J33 = m(1)*q(2) + m(2)*q(3) + m(3)*q(4);
J34 = m(1)*q(1) - m(2)*q(4) + m(3)*q(3);

step = F * [J11 J12 J13 J14; -J14 -J13 J12 J11; J31 J32 J33 J34];
step = step / norm(step);

if zeta ~= 0
    we = [q(1)*step(2) - q(2)*step(1) - q(3)*step(4) + q(4)*step(3), ...
          q(1)*step(3) + q(2)*step(4) - q(3)*step(1) - q(4)*step(2), ...
          q(1)*step(4) - q(2)*step(3) + q(3)*step(2) - q(4)*step(1)];

    q(5:7) = q(5:7) + 2*zeta*we*dt;
    g = g - q(5:7);
end

if norm(g) < 1e-6
    q(1:4) = q(1:4) + (0.5*quatvmult(q, g) - beta*step) * dt;
else
    q(1:4) = quatmult(q(1:4), deltaq(0.5*g*dt)) - beta*step * dt;
end

q(1:4) = q(1:4) / norm(q(1:4));


function dq = deltaq(hgdt)

n = norm(hgdt);
dq = [cos(n) sin(n)*hgdt/n];
