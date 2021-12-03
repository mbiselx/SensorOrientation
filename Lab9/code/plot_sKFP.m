clc
close all
clear

s_e_less    = load("data/0p1_sKFP.mat").sigma_KFP;
s_t_less    = 0.451708;
s_e_nom     = load("data/original_sKFP.mat").sigma_KFP;
s_t_nom     = 0.425634;
s_e_more    = load("data/10_sKFP.mat").sigma_KFP;
s_t_more    = 0.560567;

s_GPS       = sqrt(2) * 0.5;


fig1 = figure();
fig1.Name = "Estimated position quality";

C = colororder;

hold on
plot(1:length(s_e_less), s_e_less,              '-', 'Color', C(1,:));
plot([1, length(s_e_less)], s_t_less*ones(1,2), '--', 'Color', C(1,:));

plot(1:length(s_e_nom),  s_e_nom ,              '-', 'Color', C(2,:));
plot([1, length(s_e_nom )], s_t_nom *ones(1,2), '--', 'Color', C(2,:));

plot(1:length(s_e_more), s_e_more,              '-', 'Color', C(3,:));
plot([1, length(s_e_more)], s_t_more*ones(1,2), '--', 'Color', C(3,:));

plot([1, length(s_e_more)], s_GPS*ones(1,2),    'k--', 'LineWidth', 1.5);

annotation("textbox", [.2 .4 .4 .5], 'String', "\sigma_{GPS}", 'FitBoxToText','on', 'LineStyle', 'none')

xlabel("time [s]"), ylabel("\sigma_{XY} [m]")
grid on
title("estimated positioning quality")
legend( "est. \sigma_{XY} for \sigma_a = 0.001", "true \sigma_{XY} '' ", ...
        "est. \sigma_{XY} for \sigma_a = 0.01",  "true \sigma_{XY} '' ", ...
        "est. \sigma_{XY} for \sigma_a = 0.1",   "true \sigma_{XY} '' ", ...
        'Location','southoutside', 'NumColumns', 4)