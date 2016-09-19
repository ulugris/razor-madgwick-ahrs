function q = mat2quat(A)
% Get quaternion from rotation matrix

tr = trace(A);

if tr > 0
    S = 2*sqrt(tr + 1);

    w = 0.25*S;
    x = (A(3, 2) - A(2, 3))/S;
    y = (A(1, 3) - A(3, 1))/S;
    z = (A(2, 1) - A(1, 2))/S;
elseif A(1, 1) > A(2, 2) && A(1, 1) > A(3, 3)
    S = 2*sqrt(1.0 + A(1, 1) - A(2, 2) - A(3, 3));

    w = (A(3, 2) - A(2, 3))/S;
    x = 0.25*S;
    y = (A(1, 2) + A(2, 1))/S;
    z = (A(1, 3) + A(3, 1))/S;
elseif A(2, 2) > A(3, 3)
    S = 2*sqrt(1.0 + A(2, 2) - A(1, 1) - A(3, 3));

    w = (A(1, 3) - A(3, 1))/S;
    x = (A(1, 2) + A(2, 1))/S;
    y = 0.25*S;
    z = (A(2, 3) + A(3, 2))/S;
else
    S = 2*sqrt(1.0 + A(3, 3) - A(1, 1) - A(2, 2));

    w = (A(2, 1) - A(1, 2))/S;
    x = (A(1, 3) + A(3, 1))/S;
    y = (A(2, 3) + A(3, 2))/S;
    z = 0.25*S;
end

q = [w x y z];