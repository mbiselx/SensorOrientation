function h = plot_uncertainty(x, P, style)

% solen from stack exchange
NP = 16;
ns = 3; 

alpha  = 2*pi/NP*(0:NP+1);

ellip = x + ns*chol(P)'*[cos(alpha);sin(alpha)];

h = plot(ellip(1,:), ellip(2,:), style);

end

