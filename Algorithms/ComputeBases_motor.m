function [Null_Basis, Minimal_Basis, Perp_Basis] = ComputeBases_motor(model, N, M) 
% Computes system level bases for the parameter nullspace and its
% orthogonal complement. It also gives a minimal basis of unit vectors
% whose span is complementary to the parameter nullspace.

    num_params = model.NB*20;
    perp_dim = 0;
    null_dim = 0;
    perp_inds = cell(model.NB,1);
    null_inds = cell(model.NB,1);
    
    for i = 1:model.NB
        dim_i = size(N{i},1);
     
        perp_inds{i} = perp_dim +1 : perp_dim + dim_i;
        null_inds{i} = null_dim +1 : null_dim + 20 - dim_i;
        
        perp_dim = perp_dim + dim_i;
        null_dim = null_dim + 20 - dim_i;
    end
    
    Null_Basis    = zeros( num_params, null_dim );
    Perp_Basis    = zeros( num_params, perp_dim );
    Minimal_Basis = zeros( num_params, perp_dim );
    
    R    = cell(model.NB,1);
    
    for i = 1:model.NB
        i_inds = parameter_inds(i,model);
        parent = model.parent(i);   
        R{i} =  null(N{i}) ;

        Minimal_Basis( i_inds , perp_inds{i} ) = M{i};   
        Null_Basis   ( i_inds , null_inds{i} ) = R{i};

        if parent > 0
            X = model.Xtree{i};
            AX = Transform_Parameters(X);
            AX_motor = Transform_Parameters(model.Xtree_motor{i});
            
            parent_inds = parameter_inds(parent,model); 
            parent_inds = parent_inds(1:10);
            
            Null_Basis(parent_inds, null_inds{i} )= -[AX AX_motor*model.motor_constraint{i}]*R{i};
            Perp_Basis(i_inds,:)                  = [AX AX_motor*model.motor_constraint{i}]'*Perp_Basis(parent_inds,:);
        end
        Perp_Basis(i_inds, perp_inds{i} ) = N{i}'; 
    end
end

function inds = parameter_inds(i,model)
    inds = [(1:10) + 10*(i-1) , (1:10) + 10*(model.NB+i-1)];
end