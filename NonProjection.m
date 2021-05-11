function y = NonProjection(x, lowerBound, upperBound)
    [N, ~] = size(x);
    y = zeros(N, 1);
    for i = 1:1:N
        if x(i) < lowerBound(i)
            y(i) = lowerBound(i);
        elseif x(i) > upperBound(i)
            y(i) = upperBound(i);
        else
            y(i) = x(i);
        end
    end
end