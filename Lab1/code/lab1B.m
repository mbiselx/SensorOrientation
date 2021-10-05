addpath("functions")

clc
close all

% run lab1A to get the various sequences, if necessary
if ~exist("wn_sq", "var") || ~exist("rw_sq", "var") || ~exist("wGM_sq", "var")
    plt = false;
    lab1A;
end

%%----------------------------------------------------------
% 4. Compute the noise characteristics for each sequence by
%    a: autocorrelation (AC) function;
%    b: power-spectral-density (PSD);
%    c: Allan (or Wavelet) Variance
%%----------------------------------------------------------

disp("Computing noise characteristics...")

disp("\tby std()...")
std_wn = mean(cellfun(@std, wn_sq));
std_GM = mean(cell2mat(cellfun(@std, GM_sq, {0}, {2}, 'UniformOutput', false)), 2);
fprintf("\t\tmean std() for white noise is %.4f\n", std_wn);
fprintf("\t\tmean std() for Gauss-Markov process (T = %d) is %.4f\n", T1, std_GM(1));
fprintf("\t\tmean std() for Gauss-Markov process (T = %d) is %.4f\n", T2, std_GM(2));

disp("\tby autocorrelation...")
wnac_sq = cellfun(@autocorrelation, wn_sq, "UniformOutput", false);
rwac_sq = cellfun(@autocorrelation, rw_sq, "UniformOutput", false);
GMac_sq = cellfun(@autocorrelation, GM_sq, "UniformOutput", false);
s0_GM = mean(sqrt(reshape(real(cell2mat(GMac_sq'))(:,ceil(size(GMac_sq{1},2)/2)),2,[])),2);
T0_GM = mean(cell2mat(cellfun(@GaussMarkovCorrelationTime, GM_sq, "UniformOutput", false)),2);
fprintf("\t\tmean s from graph for Gauss-Markov process (T = %d) is %.4f\n", T1, s0_GM(1));
fprintf("\t\tmean s from graph for Gauss-Markov process (T = %d) is %.4f\n", T2, s0_GM(2));
fprintf("\t\tmean correlation time from graph for Gauss-Markov process (T = %d) is %.1f\n", T1, T0_GM(1));
fprintf("\t\tmean correlation time from graph for Gauss-Markov process (T = %d) is %.1f\n", T2, T0_GM(2));

disp("\tby PSD...")
wnpsd_sq = cellfun(@PowerSpectralDensity, wn_sq, "UniformOutput", false);
rwpsd_sq = cellfun(@PowerSpectralDensity, rw_sq, "UniformOutput", false);
GMpsd_sq = cellfun(@PowerSpectralDensity, GM_sq, "UniformOutput", false);
s1_wn = mean(sqrt(cellfun(@mean, wnpsd_sq)));
fprintf("\t\tmean s from graph for white noise is %.4f\n", s1_wn);

% Only do this if you want to crash your computer
% disp("\tby Allen Variance...")
% allandev(wn_sq{1}, "white noise");
% allandev(rw_sq{1}, "random walk");
% allandev(GM_sq{1}(1,:), "Gauss-Markov process");

%%----------------------------------------------------------
% 5. Verify graphically the changes of these characteristics for each type
%    of noise
%%----------------------------------------------------------

disp("Plotting noise characteristics evolution")

for i = 1:length(wn_sq)
    plot_noise(wn_sq{i}, wnac_sq{i}, wnpsd_sq{i}, "White Noise", 1)
    plot_noise(rw_sq{i}, rwac_sq{i}, rwpsd_sq{i}, "Random Walk", 2)
    plot_noise(GM_sq{i}(1,:), GMac_sq{i}(1,:), GMpsd_sq{i}(1,:), "Gauss-Markov (T = 2000)", 3)
    plot_noise(GM_sq{i}(2,:), GMac_sq{i}(2,:), GMpsd_sq{i}(2,:), "Gauss-Markov (T = 500)", 4)
    disp(".")
end



%%----------------------------------------------------------
% 6. Compare your findings with the values determined by the online tool for
%    noise characterization via GMWM.
%%----------------------------------------------------------

% TODO

disp("done")
