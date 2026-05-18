leftMember = sum(eye(nNodes).*x_matrix,'all');
b = [b;0];
constraints = leftMember <= b(end);
A_sym = [A_sym; jacobian(leftMember, x)];