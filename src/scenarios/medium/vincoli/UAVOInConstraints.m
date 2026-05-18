
leftMember = sym([]);
for i=2:nNodes
    if isempty(leftMember)
        leftMember = x_matrix(i,1);
    else
        leftMember = leftMember+x_matrix(i, 1);
    end
    
end
leftMember = leftMember - d;
b = [b;0];
constraints = leftMember <= b(end);
A_sym = [A_sym; jacobian(leftMember, x)];

