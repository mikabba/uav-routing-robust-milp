for i = 1:nTask
    for j = 1:nTask
        if i~=j
            leftMember = v(j)-v(i)+M*x_matrix(i+1,j+1);
            b = [b;M-c(i+1,j+1)];
            constraints = leftMember <= b(end);
            A_sym = [A_sym; jacobian(leftMember, x)];
        end
    end
end