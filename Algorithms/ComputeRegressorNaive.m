function Y = ComputeRegressorNaive(model, q, qd, qdd)
%% Y = ComputeRegressorNaive(model, q, qd, qdd)
% Computes the classical regressor Y by repeatedly calling the inverse
% dynamics routine, once for each column of Y. 
% (I call it Naive since there are more efficient ways to accomplish this 
% task, even if this way is correct.)
        
    for i = 1:model.NB
        model.I{i} = zeros(6);
    end
    
    k = 1;
    % Loop through, computing Y one column at a time
    
    % Loop through bodies
    for i = 1:model.NB
        % Then inertial parameters for each body
        for j = 1:10
            ej = zeros(10,1);
            ej(j) = 1;
            model.I{i} = inertiaVecToMat(ej); 
            % Run inverse dynamics with only a single inertial paramter set
            % to a unit value
            Y(:,k) = ID(model, q, qd, qdd);
            
            k = k+1;
        end
        model.I{i} = zeros(6);
    end

end