
addpath("functions")

clc
close all

% run lab2A to get the data
plt = false;
lab2A;

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




% filter out deterministic element of the noise
psd1     = real(PowerSpectralDensity(data1_0m.')).';
psd_fftsft = fftshift(psd1, 1);
[~, idx] = maxk(psd_fftsft(20:length(data1),:), 2); % dont take the central spike
f = (idx-1+20)*f1/length(psd1);

data1_f = [bandstop(data1_0m(:,1), [min(f(:,1))-1, max(f(:,1))+1], f1), ...
           bandstop(data1_0m(:,2), [min(f(:,1))-1, max(f(:,1))+1], f1)];

% save_to_file("LN200.txt", "", data1,',');
% save_to_file("NavchipA.txt", "", data2, ',');

% save_to_file("LN200_filtered.txt", "", data1_f,',');
% save_to_file("NavchipA.txt", "", data2, ',');

disp("done")