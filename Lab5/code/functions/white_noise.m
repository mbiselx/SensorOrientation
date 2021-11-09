function wn = white_noise(varargin)
% WHITE_NOISE creates a zero-mean white noise series of length k and std sigma
% wn = white_noise(k)
% wn = white_noise(k, sigma)
% this function is just a wrapper of randn(), it's meant for code
% legibility 

    if nargin < 2
        wn = randn(1,varargin{1});
    else 
        wn = varargin{2}.*randn(size(varargin{2},1),varargin{1});
    end

end
