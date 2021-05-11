function [X_dot, y] = AdaptiveControl(t, X)
    global a1 a2 c d wr psi mu a eJtheta L1 L2 Kp O1 O2 dt Gamma Phihat;
    % yd1 for yd(1), yd2 for yd(2)
    yd1 = X(1);
    yd2 = X(2);
    q1 = X(3);
    q2 = X(4);
    q = [q1; q2];
    q1_dot = X(5);
    q2_dot = X(6);
    q_dot = [q1_dot; q2_dot];
    
    % Calculate the reference signal
    r1 = a1 * sin(wr * t) + c + d * sin(1.5 * wr * t);
    r2 = a2 * sin(wr * t + psi) + c + d * sin(1.5 * wr * t + psi);
    r_dot = [a1 * cos(wr * t) * wr + d * cos(1.5 * wr * t) * 1.5 * wr;
            a2 * cos(wr * t + psi) * wr + d * cos(1.5 * wr * t + psi) * 1.5 * wr];
    % Calculate the derivative
    yd1_dot = - mu * yd1 + r1;
    yd2_dot = - mu * yd2 + r2;
    yd_dot = [yd1_dot; yd2_dot];
    % Calculate xd
    xd = a * eJtheta^-1 * [yd1; yd2];
    % Calculate qd
    qd_2 = acos(((xd(1) - O1)^2 + (xd(2) - O2)^2 - L1^2 - L2^2)/(2 * L1 * L2));
    qd_1 = atan2(xd(2) - O2, xd(1) - O1) - atan2(L2 * sin(qd_2), L1 + L2 * cos(qd_2));
    qd = [qd_1; qd_2];
    % Calculate xd_dot (xd = a eJtheta yd => xd_dot = a eJtheta yd_dot)
    xd_dot = a * eJtheta^-1 * [yd1_dot; yd2_dot];
    % Calculate xd_ddot (xd_ddot = a eJtheta yd_ddot, yd_ddot = -mu * yd_dot + r_dot)
    xd_ddot = a * eJtheta^-1 * (- mu * yd_dot + r_dot);
    % Calculate Jacobian for qd
    Jacod = Jacobian([qd_1, qd_2, L1, L2]);
    JacodInv = JacobianInv([qd_1, qd_2, L1, L2]);
    % Calculate qd_dot through Jacobian (qd_dot = JacodInv * xd_dot)
    qd_dot = JacodInv * xd_dot;
    % Calculate the derivative of the Jacobian
    Jacod_dot = Jacobian_dot([qd_1, qd_2, qd_dot(1), qd_dot(2), L1, L2]);
    qd_ddot = JacodInv * (xd_ddot - Jacod_dot * qd_dot);
    
    % Composite Error
    error = [q1; q2] - qd;
    error_dot = q_dot - qd_dot;
    s = error + error_dot;
    
    % Calculate Control Torque
    W = Regressor(q, q_dot, qd_dot - error, qd_ddot - error_dot);
    tau = - Kp * s + W * Phihat;
    
    % Matrix Mq, Vmq
    p1 = Phihat(1);
    p2 = Phihat(2);
    p3 = Phihat(3);
    Mq = [p1 + 2 * p3 * cos(q2), p2 + p3 * cos(q2);
        p2 + p3 * cos(q2), p2];
    Vmq = p3 * sin(q2) * [-q2_dot, -(q1_dot + q2_dot);
                          q1_dot, 0];
    q_ddot = Mq^-1 * (tau - Vmq * q_dot);    
    
    Phihat_dot = - Gamma * transpose(W) * s;
    Phihat = Phihat + Phihat_dot * dt;
    X_dot = [yd1_dot; yd2_dot; q1_dot; q2_dot; q_ddot(1); q_ddot(2)];
    
    y(1) = qd_1; % Refernce angle
    y(2) = qd_2; 
    y(3) = q1; % Real angle
    y(4) = q2;
    y(5) = tau(1); % Torque
    y(6) = tau(2);
    y(7) = Phihat(1);
    y(8) = Phihat(2);
    y(9) = Phihat(3);
end