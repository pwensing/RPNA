function [Yg, Yh] = ComputePositionRegressorNaive(model, q)
        Yg = ComputeRegressorNaive(model,q, 0*q, 0*q, model.gravity);
        Yh = zeros(0, 10*model.NB);
        for i = 1:length(q)
           qd = 0*q;
           qdd = 0*q;
           qdd(i) = 1;
           
           Yi = ComputeRegressorNaive(model, q, qd, qdd, [0 0 0]');
           
           Yh = [Yh ; Yi];
        end
end