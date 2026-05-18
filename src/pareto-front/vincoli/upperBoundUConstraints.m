for i = 1:nTask
    leftMember = u(i)-R(i);
    b = [b;-c(i+1,1)];
    constraints = leftMember <= b(end);
    A_sym = [A_sym; jacobian(leftMember, x)];
end