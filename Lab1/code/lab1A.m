addpath("functions")

clc
close all

%%----------------------------------------------------------
% Check "plt" flag
%%----------------------------------------------------------

if ~exist("plt", "var")
    plt = true
end

%%----------------------------------------------------------
% 1. Generate 3 random sequences (i.e. 'white noise') each at least 200’000 samples
%%----------------------------------------------------------

i = 3;
n = 200e3;

disp("Generating white noise sequences...")
wn_sq = cell(1,i);
for i = 1:length(wn_sq)
    wn_sq{i} = randn(1,n);
end

if (plt == true)
    figure(1)
    for i = 1:length(wn_sq)
        plot(wn_sq{i})
        hold on
    end
    hold off
    pause(.5)
end


%%----------------------------------------------------------
% 2. Use these sequences to generate accumulated noise also called 'random walk'
%%----------------------------------------------------------

disp("Calculating random walks...")
rw_sq = cellfun(@cumsum, wn_sq, "UniformOutput", false);

if (plt == true)
    figure(2)
    for i = 1:length(rw_sq)
        plot(rw_sq{i})
        hold on
    end
    hold off
    pause(.5)
end


%%----------------------------------------------------------
% 3. Use previously generated random sequences to generate 1st order Gauss-Markov
%    process for two correlation times
%%----------------------------------------------------------

T1 = 2000;
T2 = 500;

disp("Generating Gauss-Markov sequences...")
GM_sq = cellfun(@GaussMarkovProcess, wn_sq, {[T1; T2]}, "UniformOutput", false);

if (plt == true)
    figure(3)
    for i = 1:length(GM_sq)
        plot(GM_sq{i}(1,:), '-.', 'color', colormap(lines(10))(i,:))
        hold on
        plot(GM_sq{i}(2,:), '--', 'color', colormap(lines(10))(i,:))
        hold on
    end
    hold off
    pause(.5)
end


% tic, gm = GaussMarkovProcess(wn_sq{1}, T1); toc  % fast
% tic, gm0 = GaussMarkovProcess0(wn_sq{1}, T1);toc % solw
%
% figure(4) % validate that the fast method gives the correct solution by superposing it over the slow
% plot(gm, ';fast;', gm0, ';slow;')
% pause(.5)

%%----------------------------------------------------------
% Save all realizations into a text file in a column format per stochastic process, 8 digits after 0
%%----------------------------------------------------------

disp("Writing sequences to files...")
for i = 1:length(wn_sq)
    filename = sprintf("output%d.txt", i);
    header = "# WhiteNoise\tRandomWalk\tGaussMarkov1\tGaussMarkov2\n";
    save_to_file(filename, header, [wn_sq{i}', rw_sq{i}', GM_sq{i}']);
end

disp("done")
