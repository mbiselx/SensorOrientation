clc
clear

%%----------------------------------------------------------
% Generate 3 random sequences (i.e. 'white noise') each at least 200’000 samples
%%----------------------------------------------------------

wn_sq = cell(3,1);
for i = 1:length(wn_sq)
    wn_sq{i} = randn(1,200000);
end

figure(1)
for i = 1:length(wn_sq)
    plot(wn_sq{i})
    hold on
end


%%----------------------------------------------------------
% Use these sequences to generate accumulated noise also called 'random walk'
%%----------------------------------------------------------

rw_sq = cellfun(@cumsum, wn_sq, "UniformOutput", false);


%%----------------------------------------------------------
% Use previously generated random sequences to generate 1st order Gauss-Markov process
%%----------------------------------------------------------

beta = 1;

x = zeros(1, length(wn_sq{1}));
x(1) = wn_sq{1}(t);
for t = 2:length(wn_sq{1})
    x(t) = exp(-beta)*x(t-1) + wn_sq{1}(t);
end
