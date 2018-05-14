function [Null_Basis, Minimal_Basis, Perp_Basis, Perp_Basis_sym] = ComputeBases(model, N, M) 
% Computes system level bases for the parameter nullspace and its
% orthogonal complement. It also gives a minimal basis of unit vectors
% whose span is complementary to the parameter nullspace.

    num_params = model.NB*10;
    perp_dim = 0;
    null_dim = 0;
    perp_inds = cell(model.NB,1);
    null_inds = cell(model.NB,1);
    
    for i = 1:model.NB
        dim_i = size(N{i},1);
     
        perp_inds{i} = perp_dim +1 : perp_dim + dim_i;
        null_inds{i} = null_dim +1 : null_dim + 10 - dim_i;
        
        perp_dim = perp_dim + dim_i;
        null_dim = null_dim + 10 - dim_i;
    end
    
    Null_Basis    = zeros( num_params, null_dim );
    Perp_Basis    = zeros( num_params, perp_dim );
    Perp_Basis_sym    = sym(zeros( num_params, perp_dim ));
    
    Minimal_Basis = zeros( num_params, perp_dim );
    
    R    = cell(model.NB,1);
    
    for i = 1:model.NB
        i_inds = parameter_inds(i);
        parent = model.parent(i);   
        R{i} =  null(N{i}) ;

        Minimal_Basis( i_inds , perp_inds{i} ) = M{i};   
        Null_Basis   ( i_inds , null_inds{i} ) = R{i};

        if parent > 0
            X = model.Xtree{i};
            X_sym = model.Xtree_sym{i};
            AX = Transform_Parameters(X);
            AX_sym = Transform_Parameters(X_sym);
            parent_inds = parameter_inds(parent);         
            Null_Basis(parent_inds, null_inds{i} )= -AX*R{i};
            Perp_Basis(i_inds,:)                  = AX'*Perp_Basis(parent_inds,:);
            Perp_Basis_sym(i_inds,:)              = simplify(AX_sym'*Perp_Basis_sym(parent_inds,:));
        end
        Perp_Basis(i_inds, perp_inds{i} ) = rref(N{i})';
        
        NN = rref(N{i})';
        inds = find(abs(NN) < 1e-9); NN(inds) = 0;      
        inds = find(abs(NN-1) < 1e-9); NN(inds) = 1;
        inds = find(abs(NN+1) < 1e-9); NN(inds) = -1;
        
        
        Perp_Basis_sym(i_inds, perp_inds{i} ) = NN; 
    end
end

function inds = parameter_inds(i)
    inds = (1:10) + 10*(i-1);
end