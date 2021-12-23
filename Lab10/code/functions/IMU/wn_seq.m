function wn   = wn_seq(sigma, dt)
    wn = sigma ./ sqrt(dt) .* randn(size(sigma));
end
