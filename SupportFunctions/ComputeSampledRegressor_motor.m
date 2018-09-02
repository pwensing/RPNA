function Ystack = ComputeSampledRegressor_motor( model, samples)
%% Ystack = ComputeSampledRegressor_motor( model, samples)
% Compute a number of samples of the classical regressor Y
% columns represent contributions from link inertial parameters followed by
% motor inertial parameters.

    Ystack = [];
    for i = 1:samples
        qr = rand(model.NB,1);
        qdr = rand(model.NB,1);
        qddr = rand(model.NB,1);
        Y = ComputeRegressorNaive_motor(model,qr,qdr,qddr);
        Ystack = [Ystack ; Y];
    end
end
