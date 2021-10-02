function r = GaussMarkovProcess0(g, T, dt=1)
% Creates a Gauss-Markovs process from a white (gaussian) noise input
% param     g   : white noise input (
%                 expected shape: nb_seqences x t
% param     T   : correlation time of the GM process
%                 expected shape: nb_corr_time x 1
%                 NOTE: GaussMarkovProcess can handle multiple sequences OR
%                       multiple correlation times, not both. So choose.

    % correlation parameter
    f = exp(-dt./T);

    r(:,1) = g(:,1) .* ones(size(f));
    for k = 2:size(g,2)
        r(:,k) = f.*r(:,k-1) + g(:,k);
    end

end
