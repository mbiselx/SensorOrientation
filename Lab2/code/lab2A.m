addpath("functions")

clc
close all

%%----------------------------------------------------------
%% Check "plt" flag
%%----------------------------------------------------------

if ~exist("plt", "var")
    plt = true;
end

%%----------------------------------------------------------
%% load the assigned data
%%----------------------------------------------------------

addpath("imuData2021-20211011")

% assigned data: 2 (LN200/Nortrop G.)
name1 = "LN200";
[data1, f1] = readimu("imu2_LN200.imu", name1);

% assigned data: 4A (STIM318/Sensonor)
name2 = "NAVCHIP_FLT";
[data2, f2] = readimu("imu4_navchip_A.imu", name2);
name2 = "NAVCHIP";

% the assigned data is only the [Y-Z] Gyroscope data
t1 = data1(:,1)-data1(1,1);
data1 = data1(:,3:4);

t2 = data2(:,1)-data2(1,1);
data2 = data2(:,3:4);

%%----------------------------------------------------------
%% plot the data
%%----------------------------------------------------------

if plt 
    figure()
    subplot(2,1,1)
    plot(t1, data1(:,1))
    xlabel("t [s]"); ylabel('\omega_Y [rad s^{-1}]')
    axis('tight')
    title(name1)
    subplot(2,1,2)
    plot(t1, data1(:,2))
    xlabel("t [s]"); ylabel('\omega_Z [rad s^{-1}]')
    axis('tight')
end 

if plt 
    figure()
    subplot(2,1,1)
    plot(t2, data2(:,1))
    xlabel("t [s]"); ylabel('\omega_Y [rad s^{-1}]')
    axis('tight')
    title(name2)
    subplot(2,1,2)
    plot(t2, data2(:,2))
    xlabel("t [s]"); ylabel('\omega_Z [rad s^{-1}]')
    axis('tight')
end 

disp("done")
