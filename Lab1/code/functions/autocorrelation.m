function out = autocorrelation(x)
% computes autocorrelation, because I can't be bothered to install the
% signal-processing toolbox

    X = fft([x, zeros(size(x))], [], 2);
    out = fftshift(ifft( X .* conj(X), [], 2), 2);

    out = out ./ size(x,2);

end
