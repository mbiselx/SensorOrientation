% paternoster

if length(dbstack) <= 1 % the script is being run in stand-alone: clear the workspace 
    clc 
    close all
    clear
end


% set default values 
if ~exist('LabV', 'var')            % 7 : const. vel model
    LabV          = 9;              % 8 : const. acc model
                                    % 9 : uniform circle model
end
if ~exist('plt', 'var')
    plt           = false;
end
if ~exist('Iter', 'var')
    Iter          = 500;
end

addpath("functions")
addpath("data")

%% simulate 2D circular motion
% the mapping frame is defined as a NED frame
% the body frame has the x1 axis in the direction of motion 

f             = 1;             % [Hz]      sampling frequency
dt            = 1/f;            % [s]       sampling period 


% calculate trajectory to follow
trajectory_parameters       % get the trajectory parameters such as r, w, etc.. 
trajectory    = [r*cos(2*pi*(0:1/50:1));
                 r*sin(2*pi*(0:1/50:1))];


% initialization values for the Kalman Filter
initialization_parameters   % get the initialization values


%% allocate memory
% allocate memory for iterations 
innov         = cell(1, Iter);
xhat_err      = cell(1, Iter);
GPS_std       = cell(1, Iter);
KFp_std       = cell(1, Iter);
KFv_std       = cell(1, Iter);
sigma_KFP     = cell(1, Iter);
lbl           = cell(1, Iter);

% allocate memory for simulation
K             = round(2*pi*f/w);        % number of simulation steps
KK            = floor(K*f_GPS/f);       % number of expected GPS updated per simulation
estd_traj     = zeros(2, K);            % [x, y]
z_GPS         = zeros(2, KK);
z_GPS_err     = zeros(size(z_GPS));
xtilde        = zeros(size(x0,1), K);
xhat          = zeros(size(x0,1), KK);
sigma_KFPP    = zeros(1, K);
qk_KFP        = zeros(1, K);


%% do iterations 
for iter = 1:Iter
    %% (re)initialize the Kalman Filter with position & velocity estimations
    KalmanFilter('init', x0, P0);

    % allocate memory for simulation
    innov{iter}    = zeros(2, KK);
    xhat_err{iter} = zeros(4, KK);
    sigma_KFP{iter}= zeros(1, KK);

    
    %% simulation loop
    k_GPS = 0;
    kk = 0;
    for k = 1:K
        % current simulation time
        t = dt*k;
    
        % use polar coordinates to simulate the true position & velocity 
        p_t         =   r*[ cos(w*t);  sin(w*t)];
        v_t         = w*r*[-sin(w*t);  cos(w*t)];
    
        % predict current position
        [xtilde(:,k), Ptilde]= KalmanFilter('predict', F, G, Q, dt*(k-k_GPS));
    
        % estimate the current position from GPS data
        if ~rem(k, round(f/f_GPS))
            % indices shenanigans
            k_GPS           = k;                % keep track of time since last GPS update
            kk              = floor(k*f_GPS/f); % index of the current GPS update
    
            % create GPS data
            z_GPS_err(:,kk) = sigma_GPS.*randn(size(p_t));
            z_GPS(:,kk)     = p_t + z_GPS_err(:,kk);
    
            % USE THE KALMAN FILTER, LUKE
            if LabV < 9
                [xhat(:,kk), Phat, innov{iter}(:,kk)] = KalmanFilter('update', z_GPS(:,kk), H, R);
            else
                [xhat(:,kk), Phat, innov{iter}(:,kk)] = KalmanFilter('update', z_GPS(:,kk), h, H, R);
            end

            % prepare outputs 
            estd_traj(:,k)  = h(xhat(1:2, kk));
            P               = M(xhat(:,kk)) * Phat * M(xhat(:,kk)).';

            xhat_err{iter}(:,kk)  = [p_t; v_t] - m(xhat(:,kk));
            sigma_KFP{iter}(:,kk) = sqrt(sum(diag(P(1:2, 1:2))));
            
        else 
            % prepare outputs 
            estd_traj(:,k)  = h(xtilde(:, k));
            P = M(xtilde(:,k)) * Ptilde * M(xtilde(:,k)).';
        end
        sigma_KFPP(:,k) = sqrt(sum(diag(P(1:2, 1:2)))); % ugh why does Matlab not support indexing a function output?? 
      

% %         do a fun plot
% %         if ~rem(k, round(f))
%             fig = plot_everything({trajectory, p_t, estd_traj(:,k), ...
%                                     estd_traj(:,1:k), P(1:2, 1:2), ...
%                                     z_GPS(:, 1:kk)});
%             ylim([-27, 27])
%             axis manual
% 
%             if (k==1), fig.WindowState = 'maximized'; end
%             drawnow
% 
% %             makeGIF(fig, sprintf("Lab%d.gif", LabV), 0.01, (k==1))
% %         end

    end    

%     if plt || exist("fig", 'var')
%         fig = plot_everything({trajectory, p_t, estd_traj(:,end), ...
%                                 estd_traj, P(1:2, 1:2), ...
%                                 z_GPS});
%         ylim([-27, 27])
%         axis manual
%         fig.WindowState = 'normal';
%     end

    %% do tasks

    % Task 4.1 : GPS ERROR 
    GPS_std{iter} = sqrt(sum(var(z_GPS_err, 0, 2)));
    
    % Task 4.2 : true positioning and velocity quality
    KFp_std{iter}  = sqrt(sum(var(xhat_err{iter}(1:2,:), 0, 2)));
    KFv_std{iter}  = sqrt(sum(var(xhat_err{iter}(3:4,:), 0, 2)));

    fprintf(".");
end % for iteration
fprintf(newline); % display
GPS_std = cell2mat(GPS_std);
KFp_std = cell2mat(KFp_std);
KFv_std = cell2mat(KFv_std);
iinnov  = cell2mat(innov);


%% output


% Task 4.1 : GPS ERROR 
disp('<strong>Task 4.1</strong>')
m_GPS_std = mean(GPS_std);
fprintf("mean GPS std() is %f m \n", m_GPS_std)
if ((Iter <= 5) && (Iter > 1)), disp(GPS_std); end


% Task 4.2 : true positioning and velocity quality
disp('<strong>Task 4.2</strong>')
m_KFp_std = mean(KFp_std);
m_KFv_std = mean(KFv_std);
fprintf("Filtered position std() is %f m \n", m_KFp_std)
if ((Iter <= 5) && (Iter > 1)), disp(KFp_std); end
fprintf("Filtered veloctiy std() is %f m/s \n", m_KFv_std)
if ((Iter <= 5) && (Iter > 1)), disp(KFv_std); end


% Task 4.3 : estimated position quality
stb        = 50; % stabilisation threshold
disp('<strong>Task 4.3</strong>')
fprintf("KF-prediction quality at stabilization is %f m \n", sigma_KFP{end}(end))
if plt 
    fprintf("plotting evolution of KF-prediction quality...\n")

    fig1 = figure(1);
    fig1.Name = "Estimated position quality";

    for iter = 1:Iter
        plot(dt_GPS * (1:KK), sigma_KFP{iter}), hold on
        lbl{iter} = sprintf('iteration %d', iter); % prepare labels for plots
    end

    xlabel("time [s]"), ylabel("estimated positioning quality[m]")
    grid on
    legend(lbl)
end

% Task 4.4 : innovation evolution
mean_innov = mean(reshape(cell2mat(innov), 2, [], iter), 3);
stb_iinnov = cell2mat(cellfun(@(i) i(:, stb:end), innov, 'UniformOutput', false));
if plt 
    disp('<strong>Task 4.4</strong>')
    fprintf("plotting evolution of KF-innovation...\n")

    fig2 = figure(2);
    fig2.Name = "Evolution of KF-innovation";

    for iter = 1:Iter
        subplot(2,1,1)
            hold on
            h1 =      plot(dt_GPS * (1:length(innov{iter})), innov{iter}(1,:), 'b');
            h1 = [h1, plot(dt_GPS * (1:length(innov{iter})), innov{iter}(2,:), 'r')];
        subplot(2,1,2)
            hold on
            h2 =      plot(dt_GPS * (1:length(innov{iter})), innov{iter}(1,:), 'b');
            h2 = [h2, plot(dt_GPS * (1:length(innov{iter})), innov{iter}(2,:), 'r')];
    end
    
    % h1 = []; h2 = [];
    subplot(2,1,1)
        h1 = [h1, plot(dt_GPS * (1:length(mean_innov)), mean_innov(1,:), 'b-.', 'LineWidth', 1.5)]; hold on
        h1 = [h1, plot(dt_GPS * (1:length(mean_innov)), mean_innov(2,:), 'r-.', 'LineWidth', 1.5)];
        xlabel("time [s]"), ylabel("innovation [m]"), grid on
        legend(h1, {'innov_N', 'innov_E', 'mean innov_N', 'mean innov_E'});%, 'Location', 'south')
    subplot(2,1,2)
        h2 = [h2, plot(dt_GPS * (stb:length(mean_innov)), mean_innov(1,stb:end), 'b-.', 'LineWidth', 1.5)]; hold on
        h2 = [h2, plot(dt_GPS * (stb:length(mean_innov)), mean_innov(2,stb:end), 'r-.', 'LineWidth', 1.5)];
        xlim(dt_GPS * [stb, length(mean_innov)])
        xlabel("time [s]"), ylabel("innovation [m]"), grid on
    %   legend(h2, ["innov_N", "innov_E", "mean innov_N", "mean innov_E"])
end


%%

fig3 = figure(3);  % plot velocity errors
fig3.Name = "Velocity errors";
xhat_err_c2m = cell2mat(xhat_err);
mean_xhat_err = mean(reshape(xhat_err_c2m(3:4,:), 2, [], iter), 3);
h = []; l = {};
for iter = 1:Iter
    h(1) = plot(dt*(1:length(xhat_err{iter})), xhat_err{iter}(3,:), 'b', 'LineWidth', .25);
    l{1} = "v_{err N}";
    hold on
    h(2) = plot(dt*(1:length(xhat_err{iter})), xhat_err{iter}(4,:), 'r', 'LineWidth', .25);  
    l{2} = "v_{err E}";
end
h1 = plot(dt*(1:length(xhat_err{iter})), mean_xhat_err(1,:), 'b-.', 'LineWidth', 1.5);
hold on
h2 = plot(dt*(1:length(xhat_err{iter})), mean_xhat_err(2,:), 'r-.', 'LineWidth', 1.5);
xlabel("time [s]"), ylabel("veloctiy error [m/s]")
legend([h, h1, h2], [l, "mean v_{err N}", "mean v_{err E}"])
% 
% 
% fig4 = figure(4);    % plot innovation histograms
% fig4.Name = "Innovation Histogram (full)";
% h1 = []; h2 = []; l1 = []; l2 = [];
% for iter = 1:Iter
%     subplot(2,1,1)
%     h1 = histogram(innov{iter}(1,:), 'DisplayStyle', 'stairs', 'EdgeColor', 'b'); hold on
%     l1 = "innov_{N}";
%     subplot(2,1,2)
%     h2 = histogram(innov{iter}(2,:), 'DisplayStyle', 'stairs', 'EdgeColor', 'r'); hold on
%     l2 = "innov_{E}";
% end
% subplot(2,1,1)
% h1 = [h1, histogram(mean_innov(1,:), 'FaceColor', 'b')]; hold on;
% legend(h1, [l1, "mean innov_{N}"])
% subplot(2,1,2)
% h2 = [h2, histogram(mean_innov(2,:), 'FaceColor', 'r')]; hold on;
% legend(h2, [l2, "mean innov_{E}"])
% sgtitle("innovation over full experiment")
% 
% 
% fig5 = figure(5);    % plot stabilized innovation histograms 
% fig5.Name = "Innovation Histogram (stable)";
% h1 = []; h2 = []; l1 = []; l2 = [];
% for iter = 1:Iter
%     subplot(2,1,1)
%     h1 = histogram(innov{iter}(1,stb:end), 'DisplayStyle', 'stairs', 'EdgeColor', 'b'); hold on
%     l1 = "innov_{N}";
%     subplot(2,1,2)
%     h2 = histogram(innov{iter}(2,stb:end), 'DisplayStyle', 'stairs', 'EdgeColor', 'r'); hold on
%     l2 = "innov_{E}";
% end
% subplot(2,1,1)
% h1 = [h1, histogram(mean_innov(1,stb:end), 'FaceColor', 'b')]; hold on;
% legend(h1, [l1, "mean innov_{N}"])
% subplot(2,1,2)
% h2 = [h2, histogram(mean_innov(2,stb:end), 'FaceColor', 'r')]; hold on;
% legend(h2, [l2, "mean innov_{E}"])
% sgtitle("innovation once stability reached")


fig6 = figure(6);    % plot innovation histogram summed
fig6.Name = "Summed Innovation Histogram (full)";
[bl, bh] = bounds(iinnov, 2);
irange = [linspace(bl(1),bh(1)); linspace(bl(2), bh(2))];
istd   = std(iinnov, [], 2);
imean  = mean(iinnov, 2);
subplot(2,1,1)
h1 = histogram(iinnov(1,:),'Normalization', 'pdf', 'FaceColor', 'b'); hold on 
h1 = [h1, plot(irange(1,:), normpdf(irange(1,:), imean(1,:), istd(1,:)), 'k', 'LineWidth', 1.5)];
l1 = ["innov_{N}", "Normal distr."];
xlabel("innovation [m]")
legend(h1, l1)
subplot(2,1,2)
h2 = histogram(iinnov(2,:),'Normalization', 'pdf', 'FaceColor', 'r'); hold on
h2 = [h2, plot(irange(2,:), normpdf(irange(2,:), imean(2,:), istd(2,:)), 'k', 'LineWidth', 1.5)];
l2 = ["innov_{E}", "Normal distr."];
xlabel("innovation [m]")
legend(h2, l2)
sgtitle(sprintf("PDF of innovation over %d full runs", Iter))

fig7 = figure(7);    % plot innovation histogram summed
fig7.Name = "Summed Innovation Histogram (stable)";
[bl, bh] = bounds(stb_iinnov, 2);
irange = [linspace(bl(1),bh(1)); linspace(bl(2), bh(2))];
istd   = std(stb_iinnov, [], 2);
imean  = mean(stb_iinnov, 2);
subplot(2,1,1)
h1 = histogram(stb_iinnov(1,:),'Normalization', 'pdf', 'FaceColor', 'b'); hold on 
h1 = [h1, plot(irange(1,:), normpdf(irange(1,:), imean(1,:), istd(1,:)), 'k', 'LineWidth', 1.5)];
xlabel("innovation [m]")
l1 = ["innov_{N}", "Normal distr."];
legend(h1, l1)
subplot(2,1,2)
h2 = histogram(stb_iinnov(2,:),'Normalization', 'pdf', 'FaceColor', 'r'); hold on
h2 = [h2, plot(irange(2,:), normpdf(irange(2,:), imean(2,:), istd(2,:)), 'k', 'LineWidth', 1.5)];
l2 = ["innov_{E}", "Normal distr."];
xlabel("innovation [m]")
legend(h2, l2)
sgtitle(sprintf("PDF of stabilized innovation over %d runs", Iter))


fig8 = figure(8);    % innovation QQplot summed
fig8.Name = "Summed Innovation QQplot (full)";
colororder([[0, 0, 1]; [1, 0, 0]])
h = qqplot(iinnov.');
legend(h(1:2), "innov_{N}", "innov_{E}", 'Location', 'southeast')
title(sprintf("innovation over %d full runs", Iter))

fig9 = figure(9);    % innovation QQplot summed
fig9.Name = "Summed Innovation QQplot (stable)"; 
colororder([[0, 0, 1]; [1, 0, 0]])
h = qqplot(stb_iinnov.');
legend(h(1:2), "innov_{N}", "innov_{E}", 'Location', 'southeast')
title(sprintf("stable innovation over %d runs", Iter))




%%
disp("done")