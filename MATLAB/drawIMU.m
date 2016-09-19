function p = drawIMU

p.f = figure(1); clf; hold on
set(p.f, 'windowstyle', 'docked', 'currentchar', 'X', 'renderer', 'zbuffer')

% IMU scale and dimensions
k = 1.20;
x = 1.00;
y = 0.60;
z = 0.05;

% Half lengths
p.d = 0.5*k*[x y z];

% Plot three empty axes
p.v(1) = vplot(zeros(3, 1), zeros(3, 1), 1, [1.0 0.0 0.0], 2);
p.v(2) = vplot(zeros(3, 1), zeros(3, 1), 1, [0.0 0.5 0.0], 2);
p.v(3) = vplot(zeros(3, 1), zeros(3, 1), 1, [0.0 0.0 1.0], 2);

% Generate vertices for centered box
verts = ([0 0 0; 0 y 0; x y 0; x 0 0; 0 0 z; 0 y z; x y z; x 0 z]);
p.verts = k*[verts(:, 1)-x/2, verts(:, 2)-y/2, verts(:, 3)-z/2];

% Generate box plot
faces = [1 2 3 4; 5 6 7 8; 3 4 8 7; 1 2 6 5; 2 3 7 6; 1 4 8 5];
p.b = patch('faces', faces, 'vertices', verts);
set(p.b, 'facecolor', 0.7*[1 1 1], 'edgecolor', 'k')

% Generate text label
p.t = uicontrol('style', 'text', 'position', [10 10 120 70]);
set(p.t, 'horizontalalignment', 'left', 'backgroundcolor', 0.8*[1 1 1]);

% View settings
axis([-1 1 -1 1 -1 1]); view(90, 0)
set(gca, 'dataaspectratio', [1 1 1])
set(gca, 'cameraupvector', [0 0 -1], 'projection', 'perspective')
hold off; axis off


function p = vplot(v, r, s, c, w)

x = r(1) + [0 s*v(1)];
y = r(2) + [0 s*v(2)];
z = r(3) + [0 s*v(3)];

p = plot3(x, y, z, 'color', c, 'linewidth', w);
