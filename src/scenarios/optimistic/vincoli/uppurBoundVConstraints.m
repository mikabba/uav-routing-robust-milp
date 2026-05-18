for i = 1:nTask
    leftMember = v(i)-R(i);
    b = [b;-c(1,i+1)];
    constraints = leftMember <= b(end);
    A_sym = [A_sym; jacobian(leftMember, x)];
end