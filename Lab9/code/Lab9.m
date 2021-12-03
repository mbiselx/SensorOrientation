% paternoster

if length(dbstack) <= 1
    % the script is being run in stand-alone: clear the workspace 
    clc 
    close all
    clear
    disp("main")
end


% set default values 
if ~exist('LabV', 'var')
    LabV          = 8;              % 7 : const. vel model 
                                    % 8 : const. acc model
end
if ~exist('plt', 'var')
    plt           = false;
end
if ~exist('Iter', 'var')
    Iter          = 10;
end


addpath('functions')


%% simulate 2D circular motion
% the mapping frame is defined as a NED frame
% the body frame has the x1 axis in the direction of motion 

f             = 1;             % [Hz]      sampling frequency
dt            = 1/f;            % [s]       sampling period 

r             = 25;             % [m]       radius of the trajectory 
w             = pi/100;         % [rad/s]   angular speed

f_GPS         = 1;              % [Hz]      GPS update rate
dt_GPS        = 1/f_GPS;        % [s]       GPS sampling period 
sigma_GPS     = [.5; .5];       % [m]       GPS measurement noise

p0            = [     r;   0];  % [m]
sigma_p0      = [ 10;  10];     % [m]
v0            = [     0; r*w];  % [m]
sigma_v0      = [0.1; 0.1];     % [m]

switch LabV 
    case 7
        disp("Executing Lab 7")
        sigma_mov_v   = [0.05; 0.05];   % [m/s^2/sqrt(Hz)]
    case 8
        disp("Executing Lab 8")
        a0            = [-r*w^2;   0];  % [m]
        sigma_a0      = [0.1; 0.1];     % [m]
        sigma_mov_a   = [0.01; 0.01];   % [m/s^3/sqrt(Hz)]
    otherwise
        error("unknown Lab version")
end

% calculate trajectory to follow
trajectory    = [r*cos(2*pi*(0:1/50:1));
                 r*sin(2*pi*(0:1/50:1))];

% initialization values for the Kalman Filter
G = [];
q = [];
R = diag(sigma_GPS.^2);   
if ~exist("a0", 'var') || ~exist("sigma_a0", 'var') 
    % constant velocity motion model
    F  = diag(ones(1,2), 2);
    P0 = diag([sigma_p0; sigma_v0].^2);
    H = [eye(2), zeros(2)];
    x0 = [p0; v0];
else
    % constant acceleration motion model
    F  = diag(ones(1,4), 2);
    P0 = diag([sigma_p0; sigma_v0; sigma_a0].^2);
    H = [eye(2), zeros(2,4)];
    x0 = [p0; v0; a0];
end

if exist("sigma_v0", 'var') && exist('sigma_mov_v', 'var')
    % noise on the velocity
    tmp = zeros(size(F,1),2);
    tmp(3:4,:)=eye(2);
    G = [G, tmp];
    q = [q;sigma_mov_v.^2];
end
if exist("sigma_a0", 'var') && exist('sigma_mov_a', 'var')
    % noise on the velocity
    tmp = zeros(size(F,1),2);
    tmp(5:6,:)=eye(2);
    G = [G, tmp];
    q = [q;sigma_mov_a.^2];
end
Q = diag(q);
clear tmp q % clean up the workspace

% allocate memory for simulation
K             = round(2*pi*f/w);        % number of simulation steps per iteration
estd_traj     = zeros(2, K);            % [x, y]
z_GPS         = zeros(2, floor(K*f_GPS/f));
z_GPS_err     = zeros(size(z_GPS));
xtilde        = zeros(size(x0,1), K);
xhat          = zeros(size(x0,1), length(z_GPS));
sigma_KFP     = zeros(1, length(z_GPS));
sigma_KFPP    = zeros(1, K);
qk_KFP        = zeros(1, K);
innov         = cell(1, Iter);
xhat_err      = cell(1, Iter);
GPS_std       = cell(1, Iter);
KFp_std       = cell(1, Iter);
KFv_std       = cell(1, Iter);
lbl           = cell(1, Iter);


%% do iterations 
for iter = 1:Iter
    %% (re)initialize the Kalman Filter with position & velocity estimations
    KalmanFilter('init', x0, P0);

    % allocate memory for simulation
    innov{iter}    = zeros(2, length(z_GPS));
    xhat_err{iter} = zeros(4, length(z_GPS));
    
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
        [xtilde(:,k), Ptilde, Qk] = KalmanFilter('predict', F, G, Q, dt*(k-k_GPS));
    
        % estimate the current position from GPS data
        if ~rem(k, round(f/f_GPS))
            % indices shenanigans
            k_GPS           = k;                % keep track of time since last GPS update
            kk              = floor(k*f_GPS/f); % index of the current GPS update
    
            % create GPS data
            z_GPS_err(:,kk) = sigma_GPS.*randn(size(p_t));
            z_GPS(:,kk)     = p_t + z_GPS_err(:,kk);
    
            % USE THE KALMAN FILTER, LUKE
            [xhat(:,kk), Phat, innov{iter}(:,kk)] = KalmanFilter('update', z_GPS(:,kk), H, R);
    
            % prepare outputs 
            xhat_err{iter}(:,kk)  = [p_t;v_t] - xhat(1:4,kk);
            sigma_KFP(:,kk) = sqrt(sum(diag(Phat(1:2, 1:2))));
            estd_traj(:,k)  = xhat(1:2, kk);
            P               = Phat;
    
        else 
            % prepare outputs 
            estd_traj(:,k) = xtilde(1:2, k);
            P = Ptilde;
        end
        sigma_KFPP(:,k) = sqrt(sum(diag(P(1:2, 1:2)))); % ugh why does Matlab not support indexing a function output?? 
        qk_KFP(:,k)     = sqrt(sum(diag(Qk(1:2, 1:2))));
    

% % %         do a fun plot
%         if ~rem(k, round(f))
%             fig = plot_everything({trajectory, p_t, estd_traj(:,k), ...
%                                     estd_traj(:,1:k), P(1:2, 1:2), ...
%                                     z_GPS(:, 1:kk)});
%             fig.WindowState = 'maximized';
%     
% % %             Capture the plot as an image and write to GIF
%             if exist("GIFname", 'var')
%                 [imind, cm] = rgb2ind(frame2im(getframe(fig)), 256); 
%                 if k == round(f) 
%                   imwrite(imind, cm, GIFname, 'gif', 'Loopcount',inf, 'DelayTime', .05); 
%                 else 
%                   imwrite(imind, cm, GIFname, 'gif', 'WriteMode', 'append', 'DelayTime', .05); 
%                 end 
%             end
%         end
    
    end    
    if exist('fig', 'var')
        fig.WindowState = 'normal';
    end
    
    %% do tasks

    % Task 4.1
    GPS_std{iter} = sqrt(sum(var(z_GPS_err, 0, 2)));
    
    % Task 4.2   
    KFp_std{iter}  = sqrt(sum(var(xhat_err{iter}(1:2,:), 0, 2)));
    KFv_std{iter}  = sqrt(sum(var(xhat_err{iter}(3:4,:), 0, 2)));
    
    % Task 4.3   
    if plt 
        if ~exist('fig1', 'var')
            fig1 = figure();
            fig1.Name = "Estimated position quality";
        else 
            figure(fig1)
        end 
        plot(dt_GPS * (1:length(sigma_KFP)), sigma_KFP), hold on
    
    % Task 4.4 
        if ~exist('fig2', 'var')
            fig2 = figure();
            fig2.Name = "Evolution of KF-innovation";
        else 
            figure(fig2)
        end 
        subplot(2,1,1)
            h1 =      plot(dt_GPS * (1:length(innov{iter})), innov{iter}(1,:), 'b');
            hold on
            h1 = [h1, plot(dt_GPS * (1:length(innov{iter})), innov{iter}(2,:), 'r')];
        subplot(2,1,2)
            h2 =      plot(dt_GPS * (1:length(innov{iter})), innov{iter}(1,:), 'b');
            hold on
            h2 = [h2, plot(dt_GPS * (1:length(innov{iter})), innov{iter}(2,:), 'r')];
    
        lbl{iter} = sprintf('iteration %d', iter); % prepare labels for plots
    end

    fprintf(".");
end % for iteration
fprintf(newline); % display
GPS_std = cell2mat(GPS_std);
KFp_std = cell2mat(KFp_std);
KFv_std = cell2mat(KFv_std);
iinnov  = cell2mat(innov);


%% output


disp('<strong>Task 4.1</strong>')
m_GPS_std = mean(GPS_std);
fprintf("mean GPS std() is %f m \n", m_GPS_std)
if ((iter <= 5) && (iter > 1)), disp(GPS_std); end


disp('<strong>Task 4.2</strong>')
m_KFp_std = mean(KFp_std);
m_KFv_std = mean(KFv_std);
fprintf("Filtered position std() is %f m \n", m_KFp_std)
if ((iter <= 5) && (iter > 1)), disp(KFp_std); end
fprintf("Filtered veloctiy std() is %f m/s \n", m_KFv_std)
if ((iter <= 5) && (iter > 1)), disp(KFv_std); end



stb        = 50; % stabilisation threshold
mean_innov = mean(reshape(cell2mat(innov), 2, [], iter), 3);
stb_iinnov = cell2mat(cellfun(@(i) i(:, stb:end), innov, 'UniformOutput', false));
if plt
    disp('<strong>Task 4.3</strong>')
    fprintf("plotting evolution of KF-prediction quality...\n")
    if ~exist('fig1', 'var')
        fig1 = figure();
        fig1.Name = "Estimated position quality";
        plot(dt_GPS * (1:length(sigma_KFP)), sigma_KFP)
    else 
        figure(fig1)
    end
    xlabel("time [s]"), ylabel("estimated positioning quality[m]")
    grid on
    legend(lbl)
    
    disp('<strong>Task 4.4</strong>')
    fprintf("plotting evolution of KF-innovation...\n")
    if ~exist('fig2', 'var')
        fig2 = figure();
        fig2.Name = "Evolution of KF-innovation";
        h1 = []; h2 = [];
    else 
        figure(fig2)
    end 

    subplot(2,1,1)
        h1 = [h1, plot(dt_GPS * (1:length(mean_innov)), mean_innov(1,:), 'b-.', 'LineWidth', 1.5)];
        hold on
        h1 = [h1, plot(dt_GPS * (1:length(mean_innov)), mean_innov(2,:), 'r-.', 'LineWidth', 1.5)];
        xlabel("time [s]"), ylabel("innovation [m]"), grid on
        legend(h1, {'innov_N', 'innov_E', 'mean innov_N', 'mean innov_E'});%, 'Location', 'south')
    subplot(2,1,2)
        h2 = [h2, plot(dt_GPS * (stb:length(mean_innov)), mean_innov(1,stb:end), 'b-.', 'LineWidth', 1.5)];
        hold on
        h2 = [h2, plot(dt_GPS * (stb:length(mean_innov)), mean_innov(2,stb:end), 'r-.', 'LineWidth', 1.5)];
        xlim(dt_GPS * [stb, length(mean_innov)])
        xlabel("time [s]"), ylabel("innovation [m]"), grid on
    %   legend(h2, ["innov_N", "innov_E", "mean innov_N", "mean innov_E"])
else
    disp('<strong>Task 4.3</strong>')
    fprintf("KF-prediction quality at stabilization is %f m \n", sigma_KFP(end))
end

% figure(), plot(dt*(1:200), xtilde(5,:), dt*(1:200), xtilde(6,:))
% figure(), plot(dt*(1:legnth(xhat)), xhat(5,:), dt*(1:legnth(xhat)), xhat(6,:))

%%

% figure()  % plot velocity errors
% xhat_err_c2m = cell2mat(xhat_err);
% mean_xhat_err = mean(reshape(xhat_err_c2m(3:4,:), 2, [], iter), 3);
% h = []; l = [];
% % for iter = 1:Iter
% %     h(1) = plot(dt*(1:length(xhat_err{iter})), xhat_err{iter}(3,:), 'b', 'LineWidth', .25);
% %     l(1) = "v_{err N}";
% %     hold on
% %     h(2) = plot(dt*(1:length(xhat_err{iter})), xhat_err{iter}(4,:), 'r', 'LineWidth', .25);  
% %     l(2) = "v_{err E}";
% % end
% h1 = plot(dt*(1:length(xhat_err{iter})), mean_xhat_err(1,:), 'b-.', 'LineWidth', 1.5);
% hold on
% h2 = plot(dt*(1:length(xhat_err{iter})), mean_xhat_err(2,:), 'r-.', 'LineWidth', 1.5);
% xlabel("time [s]"), ylabel("veloctiy error [m/s]")
% legend([h, h1, h2], [l, "mean v_{err N}", "mean v_{err E}"])


% figure()    % plot innovation histograms
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


% figure()    % plot innovation histograms
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


% figure()    % plot innovation histogram summed
% [bl, bh] = bounds(iinnov, 2);
% irange = [linspace(bl(1),bh(1)); linspace(bl(2), bh(2))];
% istd   = std(iinnov, [], 2);
% imean  = mean(iinnov, 2);
% subplot(2,1,1)
% h1 = histogram(iinnov(1,:),'Normalization', 'pdf', 'FaceColor', 'b'); hold on 
% h1 = [h1, plot(irange(1,:), normpdf(irange(1,:), imean(1,:), istd(1,:)), 'k', 'LineWidth', 1.5)];
% l1 = ["innov_{N}", "Normal distr."];
% xlabel("innovation [m]")
% legend(h1, l1)
% subplot(2,1,2)
% h2 = histogram(iinnov(2,:),'Normalization', 'pdf', 'FaceColor', 'r'); hold on
% h2 = [h2, plot(irange(2,:), normpdf(irange(2,:), imean(2,:), istd(2,:)), 'k', 'LineWidth', 1.5)];
% l2 = ["innov_{E}", "Normal distr."];
% xlabel("innovation [m]")
% legend(h2, l2)
% sgtitle(sprintf("PDF of innovation over %d full runs", Iter))

% figure()    % plot stable innovation histogram summed
% [bl, bh] = bounds(stb_iinnov, 2);
% irange = [linspace(bl(1),bh(1)); linspace(bl(2), bh(2))];
% istd   = std(stb_iinnov, [], 2);
% imean  = mean(stb_iinnov, 2);
% subplot(2,1,1)
% h1 = histogram(stb_iinnov(1,:),'Normalization', 'pdf', 'FaceColor', 'b'); hold on 
% h1 = [h1, plot(irange(1,:), normpdf(irange(1,:), imean(1,:), istd(1,:)), 'k', 'LineWidth', 1.5)];
% xlabel("innovation [m]")
% l1 = ["innov_{N}", "Normal distr."];
% legend(h1, l1)
% subplot(2,1,2)
% h2 = histogram(stb_iinnov(2,:),'Normalization', 'pdf', 'FaceColor', 'r'); hold on
% h2 = [h2, plot(irange(2,:), normpdf(irange(2,:), imean(2,:), istd(2,:)), 'k', 'LineWidth', 1.5)];
% l2 = ["innov_{E}", "Normal distr."];
% xlabel("innovation [m]")
% legend(h2, l2)
% sgtitle(sprintf("PDF of stabilized innovation over %d runs", Iter))


figure()        % QQplot of the innovation, to check Normality 
colororder([[0, 0, 1]; [1, 0, 0]])
h = qqplot(iinnov.');
legend(h(1:2), "innov_{N}", "innov_{E}", 'Location', 'southeast')
title(sprintf("innovation over %d full runs", Iter))

figure()        % QQplot of the innovation, to check Normality 
colororder([[0, 0, 1]; [1, 0, 0]])
h = qqplot(stb_iinnov.');
legend(h(1:2), "innov_{N}", "innov_{E}", 'Location', 'southeast')
title(sprintf("stable innovation over %d runs", Iter))


% fig = figure();
% fig.Name = "Predicted position quality";
% plot(dt * (1:length(sigma_KFPP)), sigma_KFPP); hold on
% plot(dt * (1:length(qk_KFP)), qk_KFP); hold on
% xlabel("time [s]"), ylabel("error [m]")
% title("Predicted position quality")
% legend("\sigma_{predicted}", "")
% grid on





%%
disp("done")