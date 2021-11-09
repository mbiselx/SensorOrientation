% paternoster
clc 
close all
clear 

addpath('functions')

%% simulate 2D circular motion
% the mapping frame is defined as a NED frame
% the body frame has the x1 axis in the direction of motion 

f             = 10;            % [Hz]      sampling frequency
dt            = 1/f;            % [s]       sampling period 

int_method    = 'Euler';        % integration method 
GIFname       = 'Real_Eul_10Hz.gif';     % GIF output file

R             = 500;            % [m]       radius of the trajectory 
w             = pi/100;         % [rad/s]   angular speed
alpha         = pi/2;           % [rad]     initial heading

acc_cb_sigma  = 9.81 *  1e-3;               % [m/s^2] constant bias std()
acc_wn_sigma  = 9.81 * 50e-6 * sqrt(f);     % [m/s^2/sample] white noise std()
acc_noise_to_include = [1, 1];

gyr_cb_sigma  = deg2rad(10/3600);           % [rad/s] constant bias std()
gyr_GM_corrT  = 100;                        % [s]     correlation time of GMP
gyr_GM_sqrt_qk= deg2rad(5e-3)*sqrt(f*(1-exp(-2*dt/gyr_GM_corrT)));  % [rad/s/sample] GM driving sequence std()
gyr_wn_sigma  = deg2rad(1e-1)*sqrt(f/3600); % [rad/s/sample] white noise std()
gyr_noise_to_include = [1, 1, 1];


% calculate trajectory to follow
trajectory    = [R*cos(2*pi*(0:1/50:1));
                 R*sin(2*pi*(0:1/50:1));
                       2*pi*(0:1/50:1) ];

% initialize the inertial navigation function with position, velocity, 
% acceleration and attitude estimations, as well as integration method
x0            = [     R;   0];
v0            = [     0; R*w];
a0            = [-R*w^2;   0];
azth0         = alpha;
inertial_navigation('init', x0, v0, a0, azth0, int_method);


% allocate memory for simulation
K             = round(2*pi*f/w);        % number of simulation steps
estd_traj     = zeros(3, K);   % [x, y, azth]
x_err         = zeros(2, K);
v_err         = zeros(2, K);
azth_err      = zeros(1, K);


% prepare the noise sequences to add to the simulation
acc_cb        = random_bias(acc_cb_sigma * ones(2,1));
acc_wn        = white_noise(K, acc_wn_sigma * ones(2,1));
gyr_cb        = random_bias(gyr_cb_sigma);
gyr_GM        = GaussMarkovProcess(white_noise(K, gyr_GM_sqrt_qk), gyr_GM_corrT, dt);
gyr_wn        = white_noise(K, gyr_wn_sigma);

acc_noise     = acc_noise_to_include(1) .* acc_cb + ...
                acc_noise_to_include(2) .* acc_wn;
gyr_noise     = gyr_noise_to_include(1) .* gyr_cb + ...
                gyr_noise_to_include(2) .* gyr_GM + ...
                gyr_noise_to_include(3) .* gyr_wn;

% prepare the nominal sensor data
wb_n          = w * ones(1, K);
fb_n          = [        zeros(1, K); 
                 R*w.^2 * ones(1, K)];

% prepare the "true" sensor data, including  noise
fb     = fb_n + acc_noise;
wb     = wb_n + gyr_noise;

%% simulation loop
for k = 1:K
    % current simulation time
    t = dt*k;

    % use polar coordinates to simulate the true position, velocity 
    % and heading
    x_t         =   R*[ cos(w*t);  sin(w*t)];
    v_t         = w*R*[-sin(w*t);  cos(w*t)];
    azth_t      = alpha + w*t;

    % estimate the current position from IMU sensor data
    [estd_traj(1:2,k), v_est, ~, estd_traj(3,k)] = inertial_navigation(fb(:,k), wb(k), dt);

    % calculate the error in the estimation
    x_err(:,k)   = estd_traj(1:2,k) - x_t;
    v_err(:,k)   = v_est - v_t;
    azth_err(k)  = estd_traj(3,k) - azth_t;

%     do a fun plot
    if ~rem(k, round(f))
        fig = plot_everything({trajectory, x_t, estd_traj(1:2,k), estd_traj(:,1:k)}, ...
                              {dt*(1:k), fb_n(:,1:k), rad2deg(wb_n(1:k)), ...
                                         fb(:, 1:k),  rad2deg(wb(1:k))});
        sgtitle(sprintf("Simulation for f_s = %d Hz and %s integration method", f, int_method))
%         legend('Location','southwest')

%         Capture the plot as an image and write to GIF
        [imind, cm] = rgb2ind(frame2im(getframe(fig)), 256); 
        if k == round(f) 
          imwrite(imind, cm, GIFname, 'gif', 'Loopcount',inf, 'DelayTime', .05); 
        else 
          imwrite(imind, cm, GIFname, 'gif', 'WriteMode', 'append', 'DelayTime', .05); 
        end 
    end

end 

%% plot the errors 
[~, idx] = max(abs(x_err), [], 2);
x_err_max = x_err([1,2], idx');
azth_err = mod(azth_err + pi, 2*pi) - pi;

fprintf("The maximal error is [%g, %g] [m].\n", x_err_max(1, 1), x_err_max(2, 2))
fprintf("The final error is [%g, %g] [m].\n", x_err(1,K), x_err(2,K))

% fig = plot_everything({trajectory, x_t, estd_traj(:,K), estd_traj}, ...
%                       {dt*(1:K), fb_n, rad2deg(wb_n), ...
%                                  fb,  rad2deg(wb)});

figure
subplot(3,1,1)
    plot(dt*(1:K), sqrt(sum(x_err.^2)))
    grid on
    xlabel('t [s]'), ylabel('err_{x} [m]')
    legend("error in position", 'Location', 'northwest')
subplot(3,1,2)
    plot(dt*(1:K), sqrt(sum(v_err.^2)))
    grid on
    xlabel('t [s]'), ylabel('err_{v} [m/s]')
    legend("error in velocity", 'Location', 'northwest')
subplot(3,1,3)
    plot(dt*(1:K), rad2deg(azth_err))
    grid on
    xlabel('t [s]'), ylabel('err_{\alpha} [°]')
%     set(gca,'yticklabel',num2str(get(gca,'ytick')','%g'))
    legend("error in heading", 'Location', 'northwest')

sgtitle(sprintf("Errors for f_s = %d Hz and %s integration method",...
        f, int_method))
subplot(3,1,1)
title("all noise processes combined")

%%
disp("done")