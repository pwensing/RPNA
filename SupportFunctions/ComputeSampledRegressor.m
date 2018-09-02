function Ystack = ComputeSampledRegressor( model, samples,use_HandG_regressor)
%% Ystack = ComputeSampledRegressor( model, samples)
% Computes a given number of random samples of the classical regressor Y

    Ystack = [];
    for i = 1:samples
        qr = rand(model.NB,1);
        qdr = rand(model.NB,1);
        qddr = rand(model.NB,1);
        if nargin == 3 && use_HandG_regressor
            % The "Position Regressor" gives the regressor for the gravity
            % vector and the entries of the masss matrix.
            [Yg, Yh] = ComputePositionRegressorNaive(model, qr);
            Y = [Yg ; Yh];
        else
            Y = ComputeRegressorNaive(model,qr,qdr,qddr);
        end
        Ystack = [Ystack ; Y];
    end
end
