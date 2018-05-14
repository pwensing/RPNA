function Comp = UnitVectorComplementarySubspace(S)
    n = size(S,1);
    m1 = size(S,2);
    m2 = n - size(S,2);
    Comp = zeros(n,m2);
    k = 0;
    for i = 1:n
        ei = zeros(n,1);
        ei(i) = 1;
        Comp(:,k+1) = ei;
        if rank([S Comp]) == m1+k+1
            k = k+1;
            if k == m2
                break
            end
        end
    end
end
