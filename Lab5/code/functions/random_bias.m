function rb = random_bias(varargin)
%RANDOM_BIAS outputs a constant random bias signal
% this function is just a wrapper of randn(), it's meant for code
% legibility 

    rb = varargin{1} .* randn(size(varargin{1}));

end

