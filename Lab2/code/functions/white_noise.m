function wn = white_noise(varargin)
% wn = white_noise(k)
% wn = white_noise(k, sigma)
% Creates a zero-mean white noise series of length k and std sigma

    if nargin < 2
        wn = randn(1,varargin{1});
    else 
        wn = varargin{2}.*randn(1,varargin{1});
    end

end
