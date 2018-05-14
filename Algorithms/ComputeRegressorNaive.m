function Y = ComputeRegressorNaive(model, q, qd, qdd, gravity)
        
    for i = 1:model.NB
        model.I{i} = zeros(6);
    end
    
    model.gravity = gravity;
    
    k = 1;
    for i = 1:model.NB
        for j = 1:10
            ej = zeros(10,1);
            ej(j) = 1;
            model.I{i} = inertiaVecToMat(ej); 
            Y(:,k) = ID(model, q, qd, qdd);
            
            k = k+1;
        end
        model.I{i} = zeros(6);
    end

end