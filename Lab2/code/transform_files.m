
addpath("functions")

clc
close all

% run lab2A to get the data, if necessary
if ~exist("data1", "var") || ~exist("data2", "var")
    plt = false;
    lab2A;
    lab2B;
end


save_to_file("LN200.txt", "", data1,',');
save_to_file("NavchipA.txt", "", data2, ',');

disp("done")