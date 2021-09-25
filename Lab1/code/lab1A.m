addpath("functions")

clc
clear
close all

%%----------------------------------------------------------
% Generate 3 random sequences (i.e. 'white noise') each at least 200’000 samples
%%----------------------------------------------------------

wn_sq = cell(3,1);
for i = 1:length(wn_sq)
    wn_sq{i} = randn(1,200e0);
end

figure(1)
for i = 1:length(wn_sq)
    plot(wn_sq{i})
    hold on
end
hold off
pause(.5)


%%----------------------------------------------------------
% Use these sequences to generate accumulated noise also called 'random walk'
%%----------------------------------------------------------

rw_sq = cellfun(@cumsum, wn_sq, "UniformOutput", false);

figure(2)
for i = 1:length(rw_sq)
    plot(rw_sq{i})
    hold on
end
hold off
pause(.5)


%%----------------------------------------------------------
% Use previously generated random sequences to generate 1st order Gauss-Markov
% process for two correlation times
%%----------------------------------------------------------

T1 = 2000;
T2 = 500;

GM_sq = cellfun(@GaussMarkovProcess, wn_sq, {[T1; T2]}, "UniformOutput", false);

figure(3)
for i = 1:length(GM_sq)
    plot(GM_sq{i}(1,:), '-.', 'color', colormap(lines(10))(i,:))
    hold on
    plot(GM_sq{i}(2,:), '--', 'color', colormap(lines(10))(i,:))
    hold on
end
hold off
pause(.5)


%%----------------------------------------------------------
% Save all realizations into a text file in a column format per stochastic process, 8 digits after 0
%%----------------------------------------------------------

for i = 1:length(wn_sq)
    filename = sprintf("output%d.txt", i);
    header = "# WhiteNoise\tRandomWalk\tGaussMarkov1\tGaussMarkov2\n";
    save_to_file(filename, header, [wn_sq{i}', rw_sq{i}', GM_sq{i}']);
end
