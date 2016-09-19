function q = getquaternion(s)

while ~exist('q', 'var')
    if s.bytesAvailable >= 18
        % Read 18 bytes from buffer
        buffer = uint8(fread(s, 18, 'uint8'));

        % Calculate checksum
        cs = uint8(0);
        for i = 2:17
            cs = bitxor(cs, buffer(i));
        end
        
        % Cycle until a valid packet is found
        while buffer(1) ~= 255 || buffer(end) ~= cs
            cs = bitxor(cs, buffer(end));

            buffer = circshift(buffer, -1);
            buffer(end) = fread(s, 1, 'uint8');

            cs = bitxor(cs, bitcmp(buffer(1)));
        end

        % Get quaternion from byte array
        q = typecast(buffer(2:17), 'single');
    end
end

