function r = GaussMarkovProcess(g, T)
% Creates a Gauss-Markovs process from a white (gaussian) noise input
% param     g   : white noise input (
%                 expected shape: nb_seqences x t
% param     T   : correlation time of the GM process
%                 expected shape: nb_corr_time x 1
%                 NOTE: GaussMarkovProcess can handle multiple sequences OR
%                       multiple correlation times, not both. So choose.

    % correlation parameter
    f = exp(-1./T);

    % convolute the past noise and the correlation parameter
    F = [zeros(size(f,1), size(g,2)), f.^(0:size(g,2)-2)];
    G = [zeros(size(g)), g(2:size(g,2))];
    conv_ = real(ifft(fft(F, [], 2) .* fft(G, [], 2), [], 2) (:,1:length(g)));

    % do the thing
    r = g(:,1) .* f.^[0:size(g,2)-1] + sqrt(1-f.^2).*conv_;

end
