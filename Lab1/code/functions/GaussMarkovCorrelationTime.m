function T = GaussMarkovCorrelationTime(x)
    % estimates the correlation time of a Gauss-Markov process

    xx = autocorrelation(x);

    Tf = zeros(rows(x), 1);
    Tl = zeros(rows(x), 1);
    half_pt = size(x,2);

    for r = 1:rows(x)
        Tf(r) = half_pt - find(xx(r,:) > max(xx(r,:))/e, 1, 'first');
        Tl(r) = find(xx(r,:) > max(xx(r,:))/e, 1, 'last') - half_pt;
    end

    T = .5*(Tf+Tl);

end
