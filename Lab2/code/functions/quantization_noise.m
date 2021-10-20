function qn = quantization_noise(varargin)
% qn = quantization_noise(x)
% qn = quantization_noise(x, thrsh)
% simulate the effect of quantization on a singal

    if nargin > 1
        qn = varargin{2}.*round(varargin{1}./varargin{2});
    else
        qn = round(varargin{1});
    end   

end