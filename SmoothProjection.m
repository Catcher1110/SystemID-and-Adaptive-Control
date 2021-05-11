function y = SmoothProjection(Eta, lowerBound, upperBound)
    [N, ~] = size(Eta);
    y = zeros(N, 1);
    for i = 1:1:N
        y(i) = lowerBound(i) + 0.5 * (upperBound(i) - lowerBound(i)) * (1 - tanh(Eta(i)));
    end
end