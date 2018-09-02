function C = OutputMatrix_motor(L1, L2)
%%  C = OutputMatrix_motor(L1, L2)
% Generalization of OutputMatrix for the case of a link+motor
%
    C = zeros(0,20);
    for i = 1:size(L1,2)
        li = L1(:,i);
        for j = 1:size(L2,2)
            lj = L2(:,j);
            C = [C; Output_InnerProduct_motor(li,lj)];
        end
    end
    C = RangeBasis(C')';
end