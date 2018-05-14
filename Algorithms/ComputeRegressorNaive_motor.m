function Y = ComputeRegressorNaive_motor(model, q, qd, qdd, gravity)
        

    motor_model = model;
    motor_model.parent(end+1:end+model.NB) = model.parent;
    
    J = zeros(2*model.NB, model.NB);
    
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
    
    motor_model.gravity = gravity;
    
    k = 1;
    for i = 1:motor_model.NB
        for j = 1:10
            ej = zeros(10,1);
            ej(j) = 1;
            motor_model.I{i} = inertiaVecToMat(ej); 
            Y(:,k) = J'*ID(motor_model, q, qd, qdd);
            k = k+1;
        end
        motor_model.I{i} = zeros(6);
    end

end