function [Yg, Yh] = ComputePositionRegressorNaive(model, q)
%% [Yg, Yh] = ComputePositionRegressorNaive(model, q)
% Computes the regressor for the gravity vector (Yg) and the mass matrix 
% (Yh) in a configuration q.  
    
        % gravity regressor is the classical regressor for a static input.
        
        Yg = ComputeRegressorNaive(model,q, 0*q, 0*q);
        
        model.gravity = [0 0 0]';
        Yh = zeros(0, 10*model.NB);
        % Compute the regressor for the entries of H one column at a time.
        for i = 1:length(q)
           qd = 0*q;
           qdd = 0*q;
           qdd(i) = 1;
           
           % Call inverse dynamics wth only a sigle entry of qdd set to a
           % unit value.
           Yi = ComputeRegressorNaive(model, q, qd, qdd);
           
           Yh = [Yh ; Yi];
        end
end