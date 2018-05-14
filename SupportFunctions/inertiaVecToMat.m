function [ I ] = inertiaVecToMat( a )

Ibar = [a(5)  a(10) a(9) ; 
        a(10) a(6)  a(8) ; 
        a(9)  a(8)  a(7) ];
h    = [a(2)  a(3)  a(4)]';
m    = a(1);
I = [Ibar skew(h) ; skew(h)' m*eye(3)];

end

