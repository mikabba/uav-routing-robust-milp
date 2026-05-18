for i = 2:nNodes
    leftMember = sym([]);
    for j = 1:nNodes
        if i ~= j
            if isempty(leftMember)
                leftMember = x_matrix(i,j);
            else
                leftMember = leftMember+x_matrix(i, j);
            end
            
        end
    end
    
    beq = [beq;1];
    constraints = leftMember == 1;
    Aeq_sym = [Aeq_sym; jacobian(leftMember, x)];
end