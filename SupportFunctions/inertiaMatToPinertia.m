function [ Pinertia ] = inertiaMatToPinertia( I )
    h = skew(I(1:3,4:6));
    Ibar = I(1:3,1:3);
    m = I(6,6);
    Pinertia = [ 1/2*trace(Ibar)*eye(3)-Ibar h ; h' m ];
end

