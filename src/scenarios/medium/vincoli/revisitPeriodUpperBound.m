for i = 1:nTask
    leftMember = R(i);
    b = [b;R_max(i)];
    constraints = leftMember <= b(end);
    A_sym = [A_sym; jacobian(leftMember, x)];
end