
leftMember = sym([]);
for i=2:nNodes
    if isempty(leftMember)
        leftMember = x_matrix(1,i);
    else
        leftMember = leftMember+x_matrix(1, i);
    end
    
end
leftMember = leftMember - d;
b = [b;0];
constraints = leftMember <= b(end);
A_sym = [A_sym; jacobian(leftMember, x)];

