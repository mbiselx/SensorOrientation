addpath("functions")

clc
close all

% run lab1A to get the various sequences
if ~exist("wn_sq", "var")
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
disp("\tby autocorrelation...")
wnac_sq = cellfun(@autocorrelation, wn_sq, "UniformOutput", false);
rwac_sq = cellfun(@autocorrelation, rw_sq, "UniformOutput", false);
GMac_sq = cellfun(@autocorrelation, GM_sq, "UniformOutput", false);

disp("\tby PSD...")
wnpsd_sq = cellfun(@PowerSpectralDensity, wn_sq, "UniformOutput", false);
rwpsd_sq = cellfun(@PowerSpectralDensity, rw_sq, "UniformOutput", false);
GMpsd_sq = cellfun(@PowerSpectralDensity, GM_sq, "UniformOutput", false);

disp("\tby Allen Variance...")
allandev(wn_sq{1}, "white noise");
allandev(rw_sq{1}, "random walk");
allandev(GM_sq{1}(1,:), "Gauss-Markov process");

%%----------------------------------------------------------
% 5. Verify graphically the changes of these characteristics for each type
%    of noise
%%----------------------------------------------------------

disp("Plotting noise characteristics evolution")

figure()
for i = 1:length(wn_sq)
    title("White Noise")
    subplot(3,1,1)
        title("Time domain")
        plot(wn_sq{i});
        hold on
    subplot(3,1,2)
        title("Autocorrelation")
        plot(real(wnac_sq{i}));
        hold on
    subplot(3,1,3)
        title("Power-Spectral Density")
        plot(real(wnpsd_sq{i}));
        hold on
end

figure()
for i = 1:length(rw_sq)
    title("Random Walk")
    subplot(3,1,1)
        title("Time domain")
        plot(rw_sq{i});
        hold on
    subplot(3,1,2)
        title("Autocorrelation")
        plot(real(rwac_sq{i}));
        hold on
    subplot(3,1,3)
        title("Power-Spectral Density")
        plot(real(rwpsd_sq{i}));
        hold on
end


figure()
for i = 1:length(GM_sq)
    title("Gauss-Markov Process 1")
    subplot(3,1,1)
        title("Time domain")
        plot(GM_sq{i}(1,:));
        hold on
    subplot(3,1,2)
        title("Autocorrelation")
        plot(real(GMac_sq{i}(1,:)));
        hold on
    subplot(3,1,3)
        title("Power-Spectral Density")
        plot(real(GMpsd_sq{i}(1,:)));
        hold on
end


%%----------------------------------------------------------
% 6. Compare your findings with the values determined by the online tool for
%    noise characterization via GMWM.
%%----------------------------------------------------------

% TODO

disp("done")
