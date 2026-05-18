for i = 1:nTask
    leftMember = v(i)+u(i)-R(i);
    b = [b;0];
    constraints = leftMember <= b(end);
    A_sym = [A_sym; jacobian(leftMember, x)];
end