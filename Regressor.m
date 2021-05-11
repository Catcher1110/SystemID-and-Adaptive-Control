function W = Regressor(q, q_dot, sigma, sigma_dot)
    W = [sigma_dot(1), sigma_dot(2), 2 * sigma_dot(1) * cos(q(2)) + sigma_dot(2) * cos(q(2)) + sin(q(2)) * (-2 * sigma(1) * q_dot(2) - sigma(2) * (q_dot(1) + q_dot(2)));
        0, sigma_dot(1) + sigma_dot(2), sigma_dot(1) * cos(q(2)) + sigma(1) * q_dot(1) * sin(q(2))];
end