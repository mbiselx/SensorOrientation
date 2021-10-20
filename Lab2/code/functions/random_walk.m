function [rw] = random_walk(varargin)
% rw = random_walk(k)
% rw = random_walk(k, sigma)
% Creates a random walk of length k based on a zero-mean white noise with std sigma
    if nargin < 2
        rw = cumsum(randn(1,varargin{1}));
    else 
        rw = varargin{2}.*cumsum(randn(1,varargin{1}));
    end
end