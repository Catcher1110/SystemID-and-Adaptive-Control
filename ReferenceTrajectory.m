function [yd_dot, y] = ReferenceTrajectory(t, yd)
    global a1 a2 c d wr psi mu eJtheta a O1 O2 L1 L2;
    % yd1 for yd(1), yd2 for yd(2)
    yd1 = yd(1);
    yd2 = yd(2);
    
    % Calculate the reference signal
    r1 = a1 * sin(wr * t) + c + d * sin(1.5 * wr * t);
    r2 = a2 * sin(wr * t + psi) + c + d * sin(1.5 * wr * t + psi);
    
    % Calculate the derivative
    yd1dot = - mu * yd1 + r1;
    yd2dot = - mu * yd2 + r2;
    
    yd_dot = [yd1dot; yd2dot];
    
    y(1) = yd1; % 
    y(2) = yd2; % 
    
    x_ref = a * eJtheta^-1 * [yd1; yd2];
    y(3) = x_ref(1);
    y(4) = x_ref(2);
    
    q_ref2 = acos(((x_ref(1) - O1)^2 + (x_ref(2) - O2)^2 - L1^2 - L2^2)/(2 * L1 * L2));
    q_ref1 = atan2(x_ref(2) - O2, x_ref(1) - O1) - atan2(L2 * sin(q_ref2), L1 + L2 * cos(q_ref2));
    y(5) = q_ref1;
    y(6) = q_ref2;
end