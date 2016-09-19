function [A, Ap, As] = ypr2mat(z, zp, zs)
% Get rotation matrix from yaw-pitch-roll

a = z(1);
b = z(2);
g = z(3);

Ax = Rx(a);
Ay = Ry(b);
Az = Rz(g);

A = Az*Ay*Ax;

% First time derivative
if nargin > 1 && nargout > 1
    ap = zp(1);
    bp = zp(2);
    gp = zp(3);
    
    Axp = Rxp(a, ap);
    Ayp = Ryp(b, bp);
    Azp = Rzp(g, gp);
    
    Ap = Azp*Ay*Ax + Az*Ayp*Ax + Az*Ay*Axp;
end

% Second time derivative
if nargin > 2 && nargout > 2
    as = zs(1);
    bs = zs(2);
    gs = zs(3);

    Axs = Rxs(a, ap, as);
    Ays = Rys(b, bp, bs);
    Azs = Rzs(g, gp, gs);

    As = Azs*Ay*Ax + Az*Ays*Ax + Az*Ay*Axs + 2*(Azp*Ayp*Ax + Azp*Ay*Axp + Az*Ayp*Axp);
end


function A = Rx(a)
    c = cos(a); s = sin(a);
    A = [1 0 0; 0 c -s; 0 s c];

function Ap = Rxp(a, ap)
    c = cos(a); s = sin(a);
    Ap = ap*[0 0 0; 0 -s -c; 0 c -s];

function As = Rxs(a, ap, as)
    c = cos(a); s = sin(a);
    As = as*[0 0 0; 0 -s -c; 0 c -s] + ap^2*[0 0 0; 0 -c s; 0 -s -c];


function A = Ry(b)
    c = cos(b); s = sin(b);
    A = [c 0 s; 0 1 0; -s 0 c];

function Ap = Ryp(b, bp)
    c = cos(b); s = sin(b);
    Ap = bp*[-s 0 c; 0 0 0; -c 0 -s];

function As = Rys(b, bp, bs)
    c = cos(b); s = sin(b);
    As = bs*[-s 0 c; 0 0 0; -c 0 -s] + bp^2*[-c 0 -s; 0 0 0; s 0 -c];


function A = Rz(g)
    c = cos(g); s = sin(g);
    A = [c -s 0; s c 0; 0 0 1];

function Ap = Rzp(g, gp)
    c = cos(g); s = sin(g);
    Ap = gp*[-s -c 0; c -s 0; 0 0 0];

function As = Rzs(g, gp, gs)
    c = cos(g); s = sin(g);
    As = gs*[-s -c 0; c -s 0; 0 0 0] + gp^2*[-c s 0; -s -c 0; 0 0 0];
