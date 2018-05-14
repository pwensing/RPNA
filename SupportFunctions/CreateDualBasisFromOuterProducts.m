function Q = CreateDualBasisFromOuterProducts(V1, V2)
    Q = zeros(10,0);
    for i = 1:size(V1,2)
        for j = 1:size(V2,2)
            Q(:,end+1) = DualMatToVec( V1(:,i)* V2(:,j)' + V2(:,j)* V1(:,i)' );
        end
    end
end