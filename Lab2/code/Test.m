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

%% create model 
std_wn = sqrt(4.953e-8);
% std_GM = std();
% T_GM = ;

% white noise process
WN  = white_noise(length(t1), std_wn);

% % Gauss-Markov process
% g   = white_noise(length(t1), std_gm);
% GMP = GaussMarkovProcess(g, T, f1);
% 
% % Quantisation process
% QNT = quantization_noise_parameter(data1_0m(:,1));

% model
% model = quantization_noise(WN, QNT);
model = WN;
model = model';



%% analysis
disp("calculating model & noise characteristics")
% calculate noise characteristics 
ac1      = real(autocorrelation(data1_0m.')).';
psd1     = real(PowerSpectralDensity(data1_0m.')).';
[var1, tau1] = allanvar(data1_0m, 'octave', f1);

% calculate the model characteristics 
acm   = real(autocorrelation(model.')).';
psdm  = real(PowerSpectralDensity(model.')).';
[varm, taum] = allanvar(model, 'octave', f1);


if plt 
    plot_noise_parameters(t1, {data1_0m(:,1), ac1(:,1),  psd1(:,1), [tau1, var1(:,1)]}, strcat(name1, ' R_Y'), 1);
    plot_noise_parameters(t1, {model(:,1),    acm(:,1),  psdm(:,1), [taum, varm(:,1)]}, strcat(name1, ' R_Y'), 1);
    legend({'real', 'verification'})
    hold off;
end 
pause(.1)


%% filtering the sinusoidal noise out of a signal


    data1_f = [bandstop(data1_0m(:,1), [5, 10], f1), ...
               bandstop(data1_0m(:,2), [5, 10], f1)];

    % quantisation noise process
    QNT = quantization_noise_parameter(data1_0m(:,1));

    % model
    data1_f = quantization_noise(data1_f, QNT);

    % calculate filtered noise characteristics 
    acf      = real(autocorrelation(data1_f.')).';
    psdf     = real(PowerSpectralDensity(data1_f.')).';
    [varf, tauf] = allanvar(data1_f, 'octave', f1);


if plt 
    set(groot,'DefaultAxesFontSize',14)
    plot_noise_parameters(t1, {data1_0m(:,1), ac1(:,1),  psd1(:,1), [tau1, var1(:,1)]}, strcat(name1, ' R_Y'), 2);
    plot_noise_parameters(t1, {data1_f(:,1),  acf(:,1),  psdf(:,1), [tauf, varf(:,1)]}, strcat(name1, ' R_Y'), 2);
    legend({'real', 'filtered'})
    hold off;
end 
pause(.1)



















disp("done")