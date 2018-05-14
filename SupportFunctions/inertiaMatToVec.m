function [ a ] = inertiaMatToVec( I )
    h = skew(I(1:3,4:6));
    a = [I(6,6)  h(1) h(2) h(3) I(1,1) I(2,2) I(3,3) I(3,2) I(3,1) I(2,1) ]';
end

