function [Phi, Qk] = discretize_model(F, G, Q, delta_t)
%DISCRETIZE_MODEL discretizes a continuous time motion model and its
%   uncertainty propagation
%   [Phi, Qk] = discretize_model(F, G, Q, delta_t)
%   param   F       : motion model
%   param   G       : noise model
%   param   Q       : uncertainty of the motion model
%   param   delta_t : time elapsed since the last update

    nF = size(F,2);

    A = [       -F, G*Q*G.';
         zeros(nF),     F.'];

    B = expm(A*delta_t);

    Phi = B((end-nF+1):end, (end-size(F, 1)+1):end).';

    Qk  = Phi *  B(1:size(G,1), (end-size(G,1)+1):end);


end

