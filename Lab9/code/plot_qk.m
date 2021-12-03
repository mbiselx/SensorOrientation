% paternoster

callstack = dbstack;
if length(callstack) <= 1
    % the script is being run in stand-alone: clear the workspace 
    clc 
    close all
    clear
end


Iter  = 1;

fig1 = figure();
Labs = [7, 8];
fig1_handle = [];
fig1_legend = [];
for i = 1:2

    % this is Matlab abuse 
    LabV = Labs(i);
    Lab8;
    
    figure(fig1)
    fig1_handle = [fig1_handle, plot(dt*(1:length(qk_KFP)), qk_KFP)];
    fig1_legend = [fig1_legend, sprintf("Lab%d", Labs(i))];
    hold on
    xlabel("time [s]"), ylabel("error [m]")


end

title("Q_k position contribution")
legend(fig1_handle, fig1_legend)