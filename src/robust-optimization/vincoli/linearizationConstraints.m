for i = 1:nTask
    leftMember = v(i)-z;
    b = [b;0];
    constraints = leftMember <= b(end);
    A_sym = [A_sym; jacobian(leftMember, x)];
end