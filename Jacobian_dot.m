function y = Jacobian_dot(x)
    x1 = x(1);
    x2 = x(2);
    x1_dot = x(3);
    x2_dot = x(4);
    L1 = x(5);
    L2 = x(6);
    y = [-L1 * cos(x1) * x1_dot - L2 * cos(x1+x2) * (x1_dot + x2_dot), -L2*cos(x1+x2) * (x1_dot + x2_dot); 
            - L1* sin(x1) * x1_dot - L2 * sin(x1+x2) * (x1_dot + x2_dot), - L2 * sin(x1+x2) * (x1_dot + x2_dot)];
end