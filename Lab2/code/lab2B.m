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


%% II. Verify: check the parameters by producing a similar signal

%% model 1: LN200
    disp("Calculating noise model for dataset 1 ...")

    % periodic process
    psd_fftsft = fftshift(psd1, 1);
    [A, idx] = maxk(psd_fftsft(20:length(data1),:), 5); % dont take the central spike
    PER = PeriodicProcess(length(data1), idx+20, A);

    % white noise process
    WN    = white_noise(length(t1), std(data1_0m).'/2).';

    % quantisation noise process
    QNT = quantization_noise_parameter(data1_0m(:,1));

    % model
%     model1 = PER;
%     model1 = PER + WN;
    model1 = quantization_noise(WN+PER, QNT);

    % calculate the model characteristics 
    acm1   = real(autocorrelation(model1.')).';
    psdm1  = real(PowerSpectralDensity(model1.')).';
    [varm1, taum1] = allanvar(model1, 'octave', f1);

    % plot a comparison between the noise and the noise model
if plt 
    plot_noise_parameters(t1, {data1_0m(:,1), ac1(:,1),  psd1(:,1), [ tau1,  var1(:,1)]}, strcat(name1, ' R_Y'), 11);
    plot_noise_parameters(t1, {model1(:,1),  acm1(:,1), psdm1(:,1), [taum1, varm1(:,1)]}, strcat(name1, ' R_Y'), 11);
    legend({'real', 'model'})
    hold off;

    plot_noise_parameters(t1, {data1_0m(:,2), ac1(:,2),  psd1(:,2), [ tau1,  var1(:,2)]}, strcat(name1, ' R_Z'), 12);
    plot_noise_parameters(t1, {model1(:,2),  acm1(:,2), psdm1(:,2), [taum1, varm1(:,2)]}, strcat(name1, ' R_Z'), 12);
    legend({'real', 'model'})
    hold off;
end 
pause(.1)

%% model 2: NAVCHIP
    disp("calculating noise model for dataset 2...")

    % periodic process
    psd_fftsft = fftshift(psd2, 1);
    [A, idx] = maxk(psd_fftsft(20:length(data2),:), 5); % dont take the central spike
    PER = PeriodicProcess(length(data2), idx+20, A/5);
    

    % Gauss-Markov process
    T2 = [100; 50];
%     std_GM = 
    g2 = white_noise(length(t2), std(data2_0m./T2').');
    GMP = GaussMarkovProcess(g2, T2, 1/f2).';

    % white noise process
    WN    = white_noise(length(t2), std(data2_0m).'/2).';

    % random walk process
%     RW    = random_walk(length(t2), std(data2_0m).').';

    % quantisation noise process
    QNT   = quantization_noise_parameter(data2_0m(:,2));

    % model
%     model2 = PER;
%     model2 = GMP;
%     model2 = GMP + WN;
%     model2 = PER + WN;
    model2 = quantization_noise(PER+GMP+WN, QNT);

    % calculate the model characteristics 
    acm2   = real(autocorrelation(model2.')).';
    psdm2  = real(PowerSpectralDensity(model2.')).';
    [varm2, taum2] = allanvar(model2, 'octave', f2);

    % plot a comparison between the noise and the noise model
if plt 
    plot_noise_parameters(t2, {data2_0m(:,1), ac2(:,1),  psd2(:,1), [ tau2,  var2(:,1)]}, strcat(name2, ' R_Y'), 21);
    plot_noise_parameters(t2, {model2(:,1),  acm2(:,1), psdm2(:,1), [taum2, varm2(:,1)]}, strcat(name2, ' R_Y'), 21);
    legend({'real', 'model'})
    hold off;

    plot_noise_parameters(t2, {data2_0m(:,2), ac2(:,2),  psd2(:,2), [ tau2,  var2(:,2)]}, strcat(name2, ' R_Z'), 22);
    plot_noise_parameters(t2, {model2(:,2),  acm2(:,2), psdm2(:,2), [taum2, varm2(:,2)]}, strcat(name2, ' R_Z'), 22);
    legend({'real', 'model'})
    hold off;
end 
pause(.1)



disp("done")



