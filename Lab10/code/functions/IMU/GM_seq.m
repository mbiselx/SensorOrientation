function next = GM_seq(prev, sigma, beta, dt)
    if isempty(prev)
        next = wn_seq(sigma, dt);
    else 
        next = prev.*exp(-beta*dt) + sigma*sqrt( (1 - exp(-2*beta*dt)) ./ dt) .* randn(size(prev));
    end
end