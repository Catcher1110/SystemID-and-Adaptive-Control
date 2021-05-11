function y = JacobianInv(x)
    x1 = x(1);
    x2 = x(2);
    L1 = x(3);
    L2 = x(4);
    y = 1/(L1*L2*sin(x2))*[L2 * cos(x1+x2), L2 * sin(x1+x2);
         -L1 *cos(x1) - L2 * cos(x1+x2), - L1 * sin(x1) - L2 * sin(x1+x2)];
end