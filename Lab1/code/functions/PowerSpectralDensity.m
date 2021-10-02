function out = PowerSpectralDensity(x)
% computes power spectal density, because I can't be bothered to install the
% signal-processing toolbox

% TODO use windowing !!

    X = fft([x, zeros(size(x))], [], 2);
    out = fftshift(X .* conj(X)) / (pi/2 * size(x,2));

end
