function Y = ComputeRegressorNaive_motor(model, q, qd, qdd)
%%  Y = ComputeRegressorNaive_motor(model, q, qd, qdd)
% Compute the classical regress for a model that includes motor rotors
% Columns of Y first give the regressor for rigid-body link parameters,
% then followed by rotor inertial parameters
        
    % Create an new model that includes the rotors as extra bodies
    motor_model = model;
    motor_model.parent(end+1:end+model.NB) = model.parent;
    
    % This jacobian gives the joint rates of all joint (conventional +
    % motors) as [qd_conv qd_mot] = J*qd_conv
    J = zeros(2*model.NB, model.NB);
    
    % Loop through creating the rotor bodies
    for i = 1:model.NB
        motor_model.I{i} = zeros(6);
        motor_model.I{i+model.NB} = zeros(6);
        
        motor_model.Xtree{model.NB+i} = model.Xtree_motor{i};
        motor_model.jtype{model.NB+i} = model.jtype_motor{i};
        
        q(model.NB+i) = q(i) * model.gr{i};
        qd(model.NB+i) = qd(i) * model.gr{i};
        qdd(model.NB+i) = qdd(i) * model.gr{i};
        
        J(i, i) = 1;
        J(model.NB+i, i) = model.gr{i};
    end
    motor_model.NB = model.NB*2;
    
    k = 1;
    % Then carry out inverse dynamics with a single inertial parameter at a
    % time. 
    for i = 1:motor_model.NB
        for j = 1:10
            ej = zeros(10,1);
            ej(j) = 1;
            motor_model.I{i} = inertiaVecToMat(ej);
            
            % Use J' to project back onto the generalized forces for the
            % original mechanism.
            Y(:,k) = J'*ID(motor_model, q, qd, qdd);
            k = k+1;
        end
        motor_model.I{i} = zeros(6);
    end

end