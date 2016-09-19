function [a, m, g] = getpacket(s)

while ~exist('b', 'var')
    if s.bytesAvailable >= 38
        % Read 38 bytes from buffer
        buffer = uint8(fread(s, 38, 'uint8'));

        % Calculate checksum
        cs = uint8(0);
        for i = 2:37
            cs = bitxor(cs, buffer(i));
        end
        
        % Cycle until a valid packet is found
        while buffer(1) ~= 255 || buffer(end) ~= cs
            cs = bitxor(cs, buffer(end));

            buffer = circshift(buffer, -1);
            buffer(end) = fread(s, 1, 'uint8');

            cs = bitxor(cs, bitcmp(buffer(1)));
        end

        % Get accel, magnet and gyro data from byte array
        b = typecast(buffer(2:37), 'single');

        a = b(1:3)'; % LSB (raw) or g (calibrated)
        m = b(4:6)'; % LSB (raw) or Gauss (calibrated)
        g = b(7:9)'; % LSB (raw) or rad/s (calibrated)
    end
end