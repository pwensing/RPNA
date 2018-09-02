function [ a ] = inertiaMatToVec( I )
%% [ a ] = inertiaMatToVec( I )
% Converts a spatial inertia matrix to the vector of 10 inertial parameters
% a=[m, hx, hy, hz, Ixx, Iyy, Izz, Iyz, Ixz, Ixy]';


    h = skew(I(1:3,4:6));
    a = [I(6,6)  h(1) h(2) h(3) I(1,1) I(2,2) I(3,3) I(3,2) I(3,1) I(2,1) ]';
end

