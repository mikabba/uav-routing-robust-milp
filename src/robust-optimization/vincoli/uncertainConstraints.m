leftMember = weight*R - y;
b = [b; 0];
constraints = leftMember <= b(end);
A_sym = [A_sym; jacobian(leftMember, x)];

leftMember = weight_two*R - y;
b = [b; 0];
constraints = leftMember <= b(end);
A_sym = [A_sym; jacobian(leftMember, x)];

leftMember = weight_three*R - y;
b = [b; 0];
constraints = leftMember <= b(end);
A_sym = [A_sym; jacobian(leftMember, x)];