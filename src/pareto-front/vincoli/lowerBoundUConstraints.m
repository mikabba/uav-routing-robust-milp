for i = 1:nTask
    leftMember = -u(i);
    b = [b;-c(1,i+1)];
    constraints = leftMember <= b(end);
    A_sym = [A_sym; jacobian(leftMember, x)];
end