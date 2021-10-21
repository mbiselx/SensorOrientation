addpath("functions")

clc
close all

if ~exist("plt", "var")
    plt = false;
end

% run lab2A to get the data, if necessary
if ~exist("data1", "var") || ~exist("data2", "var")
    plt_tmp = plt;
    plt = false;
    lab2A;
    plt = plt_tmp;
end

%%----------------------------------------------------------
%% 4. Analyze the data
%%----------------------------------------------------------

%% I. Model: find out the parameters 

% choose regions of interest for analysis
T_roi1 = [700, 2000]; % removes the worst noise spikes
idx1 = (t1 > T_roi1(1)) & (t1 < T_roi1(2));
t1 = t1(idx1);
data1 = data1(idx1,:);

T_roi2 = [20, 1100]; % removes initialization noise & noise spike at the end
idx2 = (t2 > T_roi2(1)) & (t2 < T_roi2(2));
t2 = t2(idx2);
data2 = data2(idx2,:);

% remove the mean 
data1_0m = data1 - mean(data1);
data2_0m = data2 - mean(data2);

% calculate noise characteristics 
ac1      = real(autocorrelation(data1_0m.')).';
psd1     = real(PowerSpectralDensity(data1_0m.')).';
[var1, tau1] = allanvar(data1_0m, 'octave', f1);

ac2      = real(autocorrelation(data2_0m.')).';
psd2     = real(PowerSpectralDensity(data2_0m.')).';
[var2, tau2] = allanvar(data2_0m, 'octave', f2);


% filter out deterministic elements of the noise
psd_fftsft = fftshift(psd1, 1);
[~, idx] = maxk(psd_fftsft(20:length(data1),:), 2); % dont take the central spike
f = (idx-1+20)*f1/length(psd1);

data1f = [bandstop(data1_0m(:,1), [min(f(:,1))-1, max(f(:,1))+1], f1), ...
          bandstop(data1_0m(:,2), [min(f(:,2))-1, max(f(:,2))+1], f1)];

% calculate new filtered noise characteristics 
ac1f     = real(autocorrelation(data1f.')).';
psd1f    = real(PowerSpectralDensity(data1f.')).';
[var1f, tau1f] = allanvar(data1f, 'octave', f1);


%% II. Verify: check the parameters by producing a similar signal

%% model 1: LN200
    disp("Calculating noise model for dataset 1 ...")

    % white noise process
    std_wn = std(data1_f);
    WN    = white_noise(length(t1), std_wn.').';

    % model
    model1 = WN;

    % calculate the model characteristics 
    acm1   = real(autocorrelation(model1.')).';
    psdm1  = real(PowerSpectralDensity(model1.')).';
    [varm1, taum1] = allanvar(model1, 'octave', f1);

    % plot a comparison between the noise and the noise model
if plt 
    plot_noise_parameters(t1, {data1_0m(:,1), ac1(:,1),  psd1(:,1), [ tau1,  var1(:,1)]}, strcat(name1, ' R_Y'), 'b', 11);
    plot_noise_parameters(t1, {data1f(:,1),  ac1f(:,1), psd1f(:,1), [tau1f, var1f(:,1)]}, strcat(name1, ' R_Y'), 'c', 11);
    plot_noise_parameters(t1, {model1(:,1),  acm1(:,1), psdm1(:,1), [taum1, varm1(:,1)]}, strcat(name1, ' R_Y'), 'r:', 11);
    legend({'original', 'filtered', 'model'})
    hold off;

    plot_noise_parameters(t1, {data1_0m(:,2), ac1(:,2),  psd1(:,2), [ tau1,  var1(:,2)]}, strcat(name1, ' R_Z'), 'b', 12);
    plot_noise_parameters(t1, {data1f(:,2), ac1f(:,2),  psd1f(:,2), [tau1f, var1f(:,2)]}, strcat(name1, ' R_Z'), 'c', 12);
    plot_noise_parameters(t1, {model1(:,2),  acm1(:,2), psdm1(:,2), [taum1, varm1(:,2)]}, strcat(name1, ' R_Z'), 'r:', 12);
    legend({'original', 'filtered', 'model'})
    hold off;
end 
pause(.1)

%% model 2: NAVCHIP
    disp("calculating noise model for dataset 2...")

    % Gauss-Markov process
    T2 = [100; 50]
    std(data2_0m./T2')
    g2 = white_noise(length(t2), std(data2_0m./T2').');
    GMP = GaussMarkovProcess(g2, T2, 1/f2).';

    % white noise process
    std(data2_0m).'/2
    WN    = white_noise(length(t2), std(data2_0m).'/2).';

    % quantisation noise process
    QNT   = quantization_noise_parameter(data2_0m(:,2))

    % model
    model2 = quantization_noise(GMP+WN, QNT);

    % calculate the model characteristics 
    acm2   = real(autocorrelation(model2.')).';
    psdm2  = real(PowerSpectralDensity(model2.')).';
    [varm2, taum2] = allanvar(model2, 'octave', f2);

    % plot a comparison between the noise and the noise model
if plt 
    plot_noise_parameters(t2, {data2_0m(:,1), ac2(:,1),  psd2(:,1), [ tau2,  var2(:,1)]}, strcat(name2, ' R_Y'), 'b', 21);
    plot_noise_parameters(t2, {model2(:,1),  acm2(:,1), psdm2(:,1), [taum2, varm2(:,1)]}, strcat(name2, ' R_Y'), 'r:', 21);
    legend({'real', 'model'})
    hold off;

    plot_noise_parameters(t2, {data2_0m(:,2), ac2(:,2),  psd2(:,2), [ tau2,  var2(:,2)]}, strcat(name2, ' R_Z'), 'b', 22);
    plot_noise_parameters(t2, {model2(:,2),  acm2(:,2), psdm2(:,2), [taum2, varm2(:,2)]}, strcat(name2, ' R_Z'), 'r:', 22);
    legend({'real', 'model'})
    hold off;
end 
pause(.1)



disp("done")



