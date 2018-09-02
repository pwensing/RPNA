function [ I ] = inertiaVecToMat( a )
%% [ I ] = inertiaVecToMat( a )
% Converts a vector of 10 inertial parameters to a spatial inertia matrix
% a=[m, hx, hy, hz, Ixx, Iyy, Izz, Iyz, Ixz, Ixy]';

Ibar = [a(5)  a(10) a(9) ; 
        a(10) a(6)  a(8) ; 
        a(9)  a(8)  a(7) ];
h    = [a(2)  a(3)  a(4)]';
m    = a(1);
I = [Ibar skew(h) ; skew(h)' m*eye(3)];

end

