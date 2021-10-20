function [varargout] = GaussMarkovNoiseParams(varargin)
    % [T, variance_GM, variance_wn] = GaussMarkovNoiseParams(x, dt)
    % estimates the noise parameters of a Gauss-Markov process
    
    x = varargin{1};
    if nargin < 2
        dt = 1;
    else 
        dt = varargin{2};
    end

    xx = autocorrelation(x);

    Tf = zeros(size(x,1), 1);
    Tl = zeros(size(x,1), 1);
    half_pt = size(x,2);

    for r = 1:size(x,1)
        Tf(r) = half_pt - find(xx(r,:) > max(xx(r,:))*exp(-1), 1, 'first');
        Tl(r) = find(xx(r,:) > max(xx(r,:))*exp(-1), 1, 'last') - half_pt;
    end

    T = .5*(Tf+Tl)*dt;

    var_GM = var(x,[],2);

    var_wn = var_GM./(2*dt./T).*(1-exp(-2*dt./T));
%     var_wn = var_GM.*(2*dt./T)./(1-exp(-2*dt./T));

    varargout = {T, var_GM, var_wn};

end
