function y0 = updatePlot(p, x, y0, fr, nf, freq)

% Update
A = rotz(-y0)*quat2mat(x);
A(:, 2:3) = -A(:, 2:3);
set(p.b, 'vertices', p.verts*A');
for j = 1:3
    vupdate(p.v(j), A(:, j)*(1-p.d(j)), A(:, j)*p.d(j), 1);
end
ypr = yawpitchroll(x)*180/pi;
if fr == 1
    y0 = ypr(1)*pi/180;
end
if fr < nf
    string = sprintf('Frequency: (estimating)\n\nHeading: %0.0fº\nPitch: %0.0fº\nRoll: %0.0fº', ypr);
else
    string = sprintf('Frequency: %0.2f Hz\n\nHeading: %0.0fº\nPitch: %0.0fº\nRoll: %0.0fº', freq, ypr);
end
set(p.t, 'string', string)
figure(p.f)
drawnow


function A = rotz(g)

c = cos(g); s = sin(g);
A = [c -s 0; s c 0; 0 0 1];


function vupdate(p, v, r, s)

set(p, 'xdata', r(1) + [0 s*v(1)])
set(p, 'ydata', r(2) + [0 s*v(2)])
set(p, 'zdata', r(3) + [0 s*v(3)])
