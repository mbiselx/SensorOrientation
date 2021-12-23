function [varargout] = simulate_GNSS_noise(step, varargin)
%SIMULATE_GPS_NOISE Summary of this function goes here
%   Detailed explanation goes here
persistent GNSS_wn_sigma

    switch step
        case 'init'
            GNSS_wn_sigma = varargin{1};
    
        case 'get'
            varargout = {GNSS_wn_sigma * randn(2,1)};
    
    end 
end

