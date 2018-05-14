function bool = IsUnidentifiable(model,N, i, k)
    pi = zeros(10,1);
    pi(k) = 1;
    bool = true;
    
    while i > 0
        if norm( N{i}* pi) > eps^.75
            bool = false;
            return
        end
        X  = model.Xtree{i} ;
        pi = inertiaMatToVec( X' * inertiaVecToMat(pi) * X ) ;
        i = model.parent(i);
    end
    