function RazorAHRS(port, alg) %#ok<*AND2>

addpath('./Rotations')

% Select serial port
if nargin < 1
    port = 'COM3';
end

% Select Kalman (K), Madgwick (M), or Madgwick-on-board (MoB) method
if nargin < 2
    alg = 'MoB';
end

% Data rate (time step in s)
h = 0.01;

% Set AHRS algorithm parameters
switch alg
    case 'K'
        % Variance exponents for accel, magnet, gyro
        X = KalmanVar([6.0 0.5 2.4], h);
    case 'M'
        % Beta and Zeta
        X = [0.0300 0.0025];
end

% Initialize Razor IMU
s = initRazor(port);
if strcmp(alg, 'MoB')
    fprintf(s, '#oqb');  % Quaternion from on-board algorithm
else
    fprintf(s, '#oscb'); % Calibrated sensor data
end

% Initialize graphics
p = drawIMU;

% Initial orientation
if ~strcmp(alg, 'MoB')
    [a, m, g0] = getpacket(s);
    [x, P] = initcond(a, m);
end

% Main loop (press ESC to exit)
y0 = 0;
fr = 0;
nf = 500;
t0 = tic;
fl = 1e-3;
freq = 1 / h;
t = zeros(nf, 1);
syncRazor(s)
while ishandle(p.f) & abs(get(p.f, 'currentchar')) ~= 27
    fr = fr + 1;
    switch alg
        case 'K'
            [a, m, g] = getpacket(s);
            [x, P] = Kalman(x, P, a, m, 0.5*(g + g0), h, X);
            g0 = g;
        case 'M'
            [a, m, g] = getpacket(s);
            x = Madgwick(x, a, m, 0.5*(g + g0), h, X);
            g0 = g;
        case 'MoB'
            x = getquaternion(s);
    end

    t = circshift(t, -1);
    t(end) = toc(t0);

    if fr >= nf
        freq = fl * (nf-1) / (t(end)-t(1)) + (1-fl) * freq;
    end

    if fr == 1 || ~mod(fr, 4)
        y0 = updatePlot(p, x, y0, fr, nf, freq);
    end
end

fclose(s);
delete(s);
