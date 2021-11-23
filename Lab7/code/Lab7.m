% paternoster
clc 
close all
clear 

addpath('functions')
set(groot,'DefaultLineLineWidth',.5)

%% simulate 2D circular motion
% the mapping frame is defined as a NED frame
% the body frame has the x1 axis in the direction of motion 

f             = 5;              % [Hz]      sampling frequency
dt            = 1/f;            % [s]       sampling period 

GIFname       = 'Real_Eul_10Hz.gif';     % GIF output file

r             = 25;             % [m]       radius of the trajectory 
w             = pi/100;         % [rad/s]   angular speed

f_GPS         = .5;             % [Hz]      GPS update rate
dt_GPS        = 1/f_GPS;        % [s]       GPS sampling period 
sigma_GPS     = [.5; .5];       % [m]       GPS measurement noise

sigma_mov     = [0.05; 0.05];   % [m/s^2/Hz]
% sigma_mov     = 0.06 * [1;1];      % [m/s^2/Hz]
% sigma_mov     = r*w/dt/sqrt(2*f)*[1; 1]   % [m/s^2/Hz]

Iter          = 100;
innov         = cell(1, Iter);
GPS_std       = cell(1, Iter);
KF_std        = cell(1, Iter);

% calculate trajectory to follow
trajectory    = [r*cos(2*pi*(0:1/50:1));
                 r*sin(2*pi*(0:1/50:1))];


for iter = 1:Iter
    % initialize the KalmanFilter function with position & velocity estimations
    x0            = [     r;   0];
    v0            = [     0; r*w];
    P0            = diag([10, 10, .1 .1].^2);
    F             = [zeros(2), eye(2); zeros(2, 4)];
    Q             = diag(sigma_mov.^2);
    R             = diag(sigma_GPS.^2);
    H             = [eye(2), zeros(2)];
    KalmanFilter('init', [x0; v0], P0);
    
    
    % allocate memory for simulation
    K             = round(2*pi*f/w);        % number of simulation steps
    estd_traj     = zeros(2, K);            % [x, y]
    xtilde        = zeros(4, K);
    x_GPS         = zeros(2, floor(K*f_GPS/f));
    x_GPS_err     = zeros(2, length(x_GPS));
    xhat          = zeros(4, length(x_GPS));
    xhat_err      = zeros(2, length(x_GPS));
    sigma_KFP     = zeros(1, length(x_GPS));
    innov{iter}   = zeros(2, length(x_GPS));
    
    
    % simulation loop
    k_GPS = 0;
    kk = 0;
    for k = 1:K
        % current simulation time
        t = dt*k;
    
        % use polar coordinates to simulate the true position & velocity 
        x_t         =   r*[ cos(w*t);  sin(w*t)];
        v_t         = w*r*[-sin(w*t);  cos(w*t)];
    
        % predict current position
        [xtilde(:,k), Ptilde] = KalmanFilter("predict", dt*(k-k_GPS), F, Q);
    
        % estimate the current position from GPS data
        if ~rem(k, round(f/f_GPS))
            k_GPS           = k;
            kk              = floor(k*f_GPS/f);
    
            x_GPS_err(:,kk) = sigma_GPS.*randn(2,1);            % Task 4.1
            x_GPS(:,kk)     = x_t + x_GPS_err(:,kk);
    
            [xhat(:,kk), Phat, innov{iter}(:,kk)] = KalmanFilter("update", x_GPS(:,kk), R, H); % Task 4.4
    
            xhat_err(:,kk)  = x_t - xhat(1:2,kk);               % Task 4.2
            sigma_KFP(:,kk) = sqrt(sum(diag(Phat(1:2, 1:2))));  % Task 4.3
    
            estd_traj(:,k)  = xhat(1:2, kk);
            P               = Phat;
    
        else 
            estd_traj(:,k) = xtilde(1:2, k);
            P = Ptilde;
        end
    

% %         do a fun plot
%         if ~rem(k, round(f))
%             fig = plot_everything({trajectory, x_t, estd_traj(:,k), ...
%                                     estd_traj(:,1:k), P(1:2, 1:2), ...
%                                     x_GPS(:, 1:kk)});
%             fig.WindowState = 'fullscreen';
%     
% %             Capture the plot as an image and write to GIF
%             [imind, cm] = rgb2ind(frame2im(getframe(fig)), 256); 
%             if k == round(f) 
%               imwrite(imind, cm, GIFname, 'gif', 'Loopcount',inf, 'DelayTime', .05); 
%             else 
%               imwrite(imind, cm, GIFname, 'gif', 'WriteMode', 'append', 'DelayTime', .05); 
%             end 
%         end
    
    end 
    
    fig.WindowState = 'normal';
    
    % do tasks

    % Task 4.1
    GPS_std{iter} = sqrt(sum(var(x_GPS_err, 0, 2)));
    
     % Task 4.2   
    KF_std{iter} = sqrt(sum(var(xhat_err, 0, 2)));
    
    
     % Task 4.3   
    if ~exist('fig1', 'var')
        fig1 = figure();
        fig1.Name = "Estimated position quality";
    else 
        figure(fig1)
    end 
    plot(dt_GPS * (1:length(sigma_KFP)), sigma_KFP), hold on
%     xlabel("time [s]"), ylabel("estimated positioning quality[m]")
    
    
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
%         xlabel("time [s]"), ylabel("innovation [m]")
    subplot(2,1,2)
        stb = 25; % stabilisation threshold
        h2 =      plot(dt_GPS * (stb:length(innov{iter})), innov{iter}(1,stb:end), 'b');
        hold on
        h2 = [h2, plot(dt_GPS * (stb:length(innov{iter})), innov{iter}(2,stb:end), 'r')];
%         xlabel("time [s]"), ylabel("innovation [m]")


    fprintf(".");
end % for iteration
fprintf(newline); % display
GPS_std = cell2mat(GPS_std);
KF_std = cell2mat(KF_std);

%%


disp('<strong>Task 4.1</strong>')
m_GPS_std = mean(GPS_std);
fprintf("mean GPS std() is %f\n", m_GPS_std)
% disp(GPS_std)

disp('<strong>Task 4.2</strong>')
m_KF_std = mean(KF_std);
fprintf("Filtered position std() is %f\n", m_KF_std)
% disp(KF_std)

disp('<strong>Task 4.3</strong>')
fprintf("plotting evolution of KF-prediction quality...\n")
if ~exist('fig1', 'var')
    fig1 = figure();
    fig1.Name = "Estimated position quality";
    plot(dt_GPS * (1:length(sigma_KFP)), sigma_KFP)
else 
    figure(fig1)
end
lbl = cell(1, Iter);
for iter = 1:Iter
    lbl{iter} = sprintf('iteration %d', iter);
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
mean_innov = mean(reshape(cell2mat(innov), 2, [], iter), 3);
subplot(2,1,1)
    h1 = [h1, plot(dt_GPS * (1:length(mean_innov)), mean_innov(1,:), 'b-.', 'LineWidth', 1.5)];
    hold on
    h1 = [h1, plot(dt_GPS * (1:length(mean_innov)), mean_innov(2,:), 'r-.', 'LineWidth', 1.5)];
    xlabel("time [s]"), ylabel("innovation [m]")
    grid on
    legend(h1, {'innov_N', 'innov_E', 'mean innov_N', 'mean innov_E'});%, 'Location', 'south')
        
subplot(2,1,2)
    stb = 25; % stabilisation threshold
    h2 = [h2, plot(dt_GPS * (stb:length(mean_innov)), mean_innov(1,stb:end), 'b-.', 'LineWidth', 1.5)];
    hold on
    h2 = [h2, plot(dt_GPS * (stb:length(mean_innov)), mean_innov(2,stb:end), 'r-.', 'LineWidth', 1.5)];
    xlabel("time [s]"), ylabel("innovation [m]")
    grid on
%   legend(h2, ["innov_N", "innov_E", "mean innov_N", "mean innov_E"], 'Location', 'westoutside')


%%
disp("done")