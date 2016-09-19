function syncRazor(s)

% Stop data streaming
fprintf(s, '#o0');

% Clear serial port buffer
while s.bytesAvailable > 0
    fread(s, s.bytesAvailable);
    pause(0.05)
end

% Restart data streaming
fprintf(s, '#o1');