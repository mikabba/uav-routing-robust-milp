for i = 1:nTask
    leftMember = -v(i);
    b = [b;-c(i+1,1)];
    constraints = leftMember <= b(end);
    A_sym = [A_sym; jacobian(leftMember, x)];
end