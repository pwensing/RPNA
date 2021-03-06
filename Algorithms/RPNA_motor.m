function [N, M, V , C] = RPNA_motor(model, free_base)
%% [N, M, V , C] = RPNA_motor(model, free_base)
% Recursive parameter nullspace algorithm for a system with motor rotors.
% The outputs of the algorithm are
% N: cell array for the nullspace descriptors characterizing undetectable
%    inertial transfers across each joint
% M: Basis for minimal paramers (sometimes called representative base
%    parameters) of each link&rotor
% V: Attainable velocity span basis for each link&rotor
% C: Null(C) gives the unidentifiable parameters of each link&rotor
%
% See the accompanying paper for further detail.
%

    if nargin == 1
        free_base = 0;
    end

    EmptyCells = cell(model.NB,1); 
    N   = EmptyCells;   C   = EmptyCells;   O       = EmptyCells;
    M   = EmptyCells;   V   = EmptyCells;   
    V_J = EmptyCells;   V_motor = EmptyCells;
    
    for i =1:model.NB
        [~, Si] = jcalc( model.jtype{i}, 0 );
        [~, Si_motor] = jcalc( model.jtype_motor{i}, 0 );
        
        n_i = size(Si, 2);
        for j = 1:n_i
            % Create joint axis for motor adding in gearing effects
            Si_motor(:,j) = Si_motor(:,j) * model.gr{i}(j);
        end
        
        p = model.parent(i);
        if p == 0
             % If you want to mimic a free base, initialize the bases
            % for the velocity and outer product spans to represnet the
            % full space. Else seed with info from the fixed base.
            if free_base
                Vp = eye(6);
                Cp = eye(10);
            else
                Vp =  get_gravity(model);
                Cp = zeros(0,10);
            end
        else
            Vp = V{p};
            Cp = C{p};
        end
        
        % Propoagate the spans across the link
        V_J{i} = model.Xtree{i} * Vp;
        
        % Propagate the jointly attainable velocity for the link and motor
        % across from the predecessor
        V_motor{i} = [model.Xtree{i} ; model.Xtree_motor{i} ] * Vp;
        
        % Sets of rates of change with joint motions
        crmSet = cell( n_i, 1);
        crmSet_motor = cell( n_i, 1);
        paramRateSet_motor     = cell( n_i, 1);
        for k= 1:size(Si,2)
            % Velocity rates of change with joint angle
            crmSet{k} = crm(Si(:,k) );
            
            % Jointly attainable link/motor velocity rates of change
            crmSet_motor{k} = [crm(Si(:,k)) zeros(6) ; ...
                               zeros(6) crm(Si_motor(:,k))];
            
            % Inertail parameters rates of change for link/motor
            paramRateSet_motor{k} = [Rate_Parameters( Si(:,k) ) zeros(10); ...
                                     zeros(10) Rate_Parameters( Si_motor(:,k) )];
        end
        
        % Propagate velocity span across joint
        V{i} = RangeBasis([ SwitchedControllabilityMatrix( crmSet , V_J{i} ) Si]); 
        
        % Propage jointly attainable link/motor velocity span across joint
        V_motor{i} = SwitchedControllabilityMatrix( crmSet_motor , V_motor{i} );
        V_motor{i} = RangeBasis([V_motor{i} [Si ; Si_motor]]);
        
        % Create kinetic energy observability for residuals transformed to
        % the predecessor
        Cj = [Cp*Transform_Parameters( model.Xtree{i} ) ...
                Cp*Transform_Parameters( model.Xtree_motor{i} ) ];
        
        O{i} = SwitchedObservabilityMatrix(Cj, paramRateSet_motor );
        
        % Compute nullspace descriptor for joint i
        N{i} = [OutputMatrix_motor(V_motor{i}, [Si ; Si_motor]) ]; 
        for k = 1:n_i
            N{i} = [N{i} ; O{i} * paramRateSet_motor{k}];
        end
        
        % Add in the motor constraint projector (e.g. to enforce rotational 
        % symmetry of the motor components)
        N{i}(:,11:20) = N{i}(:,11:20) * model.motor_constraint{i};
        
        N{i} = RangeBasis(N{i}')'; 
        
        C{i} = [Cj(:,1:10) ; N{i}(:,1:10)];
        
        M{i} = UnitVectorComplementarySubspace( null(N{i}) );
    end
end