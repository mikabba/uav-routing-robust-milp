leftMember = d;
b = [b;nv];
constraints = leftMember <= b(end);
A_sym = [A_sym; jacobian(leftMember,x)];