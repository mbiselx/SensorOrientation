function [p] = simulate_GNSS(x_t)
%SIMULATE_GNSS Summary of this function goes here
%   Detailed explanation goes here

    p = x_t(4:5) + simulate_GNSS_noise('get');
end

