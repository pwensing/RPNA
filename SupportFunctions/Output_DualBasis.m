function C = Output_DualBasis(Q)
    % Consider parameters a and dual parameters d with matrix
    % representations I and K
    % By construction of the dual basis:
    % tr(I * K ) = d'*a
    
    C = RangeBasis(Q)';
end