function s = initRazor(port)

s = instrfind;
if ~isempty(s)
    fclose(s);
    delete(s);
end

t0 = tic;
s = serial(port, 'baudrate', 57600);
fopen(s);

fscanf(s);
fprintf('Boot delay: %0.2fs\n', toc(t0));
