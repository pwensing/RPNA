function N = SwitchedObservabilityMatrix(C,A)
    AT = A;
    for i = 1:length(A)
        AT{i} = A{i}';
    end
    N = SwitchedControllabilityMatrix(AT,C')';
end
    