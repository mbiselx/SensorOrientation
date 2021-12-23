% paternoster

if length(dbstack) <= 1 % the script is being run in stand-alone: clear the workspace 
    clc 
    close all
    clear all
end

addpath("functions\");
addpath("functions\GNSS");
addpath("functions\IMU");
addpath("functions\INS");
addpath("functions\KalmanFilter");
addpath("functions\plotting");

%% trajectory parameters
r             = 500;                        % [m]       radius of the trajectory 
psi           = 0;                          % [rad]     initial angle
w             = pi/100;                     % [rad/s]   angular speed
alpha         = pi/2;

% calculate trajectory to follow
trajectory    = [r*cos(2*pi*(0:1/200:1));
                 r*sin(2*pi*(0:1/200:1));
                       2*pi*(0:1/200:1) ];

%% timing 
f             = 100;                        % [Hz]      sampling frequency
dt            = 1/f;                        % [s]       sampling period 


%% sensor noise parameters
g             = 9.81;

acc_wn_sigma  =   50e-6 *g;                 % [m/s^2/sqrt(Hz)] white noise std()
ac1_GM_corrT  = 60;                         % [s]       correlation time of GMP
ac1_GM_sigma  =  200e-6 *g;                 % [m/s^3/sqrt(Hz)] GM PSD level std()
ac1_GM_0      = -100e-6 *g;                 % [m/s^2]   initial GM value
ac2_GM_corrT  = 60;                         % [s]       correlation time of GMP
ac2_GM_sigma  =  200e-6 *g;                 % [m/s^3/sqrt(Hz)] GM PSD level std()
ac2_GM_0      = +200e-6 *g;                 % [m/s^2]   initial GM value

gyr_wn_sigma  = deg2rad(1e-1)/sqrt(3600);   % [rad/s/sqrt(Hz)] white noise std()
gyr_GM_corrT  = 30;                         % [s]       correlation time of GMP
gyr_GM_sigma  = deg2rad(1e-2);              % [rad/s/sqrt(Hz)] GM PSD level std() 
gyr_GM_0      = 0;                          % [rad/s]   initial GM value
gyr_cb_0      = deg2rad(-400/3600);         % [rad/s]   constant bias 

simulate_IMU_noise("init", {gyr_wn_sigma, {[], gyr_cb_0}, {gyr_GM_sigma, 1/gyr_GM_corrT, gyr_GM_0}}, ...
                           {acc_wn_sigma, {ac1_GM_sigma, 1/ac1_GM_corrT, ac1_GM_0}}, ...
                           {acc_wn_sigma, {ac2_GM_sigma, 1/ac2_GM_corrT, ac2_GM_0}});

f_GPS         = .5;                         % [Hz]      GPS update rate
dt_GPS        = 1/f_GPS;                    % [s]       GPS sampling period 
GPS_wn_sigma  = 1;                          % [m]       GPS measurement noise

simulate_GNSS_noise('init', GPS_wn_sigma);


%% allocate memory

K             = round(2*pi*f/w);            % number of simulation steps
KK            = ceil(K*f_GPS/f);           % number of expected GPS updates

wb_n          = zeros(1, K);
wb            = zeros(1, K);
wb_c          = zeros(1, K);
fb_n          = zeros(2, K);
fb            = zeros(2, K);
fb_c          = zeros(2, K);

z_GPS         = zeros(2, KK);

x1_t          = zeros(5, K);
x1_tilde      = zeros(5, K);
x1_hat        = zeros(5, K);
sigma_p_hat   = zeros(1, K);


%% initialization values 

% inertial navigation
azth0         = alpha + deg2rad(3);
v0            = [0-2; w*r-1];
z_GPS(:,1)    = simulate_GNSS([azth0; v0; r; 0]);

inertial_navigation('init', [azth0; v0; z_GPS(:,1)], "Trapezoid");

% Kalman Filter
% motion model
betas         = 1./[gyr_GM_corrT; ac1_GM_corrT; ac2_GM_corrT];% * 1e2;
Q             = f * diag([gyr_wn_sigma^2, acc_wn_sigma^2, acc_wn_sigma^2, ...
                        2*gyr_GM_sigma^2*betas(1), ...
                        2*ac1_GM_sigma^2*betas(2), ...
                        2*ac2_GM_sigma^2*betas(3)]); % multiply by f to go from PSD to samples
% observation equation
H             = [zeros(2, 3), eye(2), zeros(2, 4)];
R             = diag([GPS_wn_sigma^2, GPS_wn_sigma^2]);
% initial uncertainty 
P0            = diag([deg2rad(2), 5, 5, 10, 10, ...
                      deg2rad(5e-2), deg2rad(1e-2), 300e-6*g, 300e-6*g].^2);
% P0            = diag([deg2rad(2), 5, 5, 10, 10]);    % only tracking Nav satates 
sigma_p       = @(P) sqrt(sum(diag(P(4:5, 4:5))));
if size(P0,1) < 9 
    disp("Errors are not being tracked")
    Q         = Q(1:3, 1:3);
    H         = H(:, 1:5);
    betas     = [];
end
KalmanFilter('init', zeros(size(P0,1),1), P0);


% error correction
wb_err_hat    = 0;
fb_err_hat    = [0;0];


%% simulation parameters 
% use_EKF       = false;
use_EKF       = true;

% use_err_CL    = false;
use_err_CL    = true;

if use_EKF
    vis_mode = 'follow';
    KF_mode  = "EKF";
else
    vis_mode = 'global';
    KF_mode  = "LKF";
end
if use_err_CL
    CL_mode = "closed loop";
else
    CL_mode = "no";
end
simtitle = sprintf("Simulation using %s and %s sensor error correction", KF_mode, CL_mode);
errtitle = sprintf("Errors using %s and %s sensor error correction", KF_mode, CL_mode);
disp(simtitle)


%% simulation (finally)
k_GPS         = 1;
kk            = 1;
fig1          = figure(101);
fig2          = figure(102);
% fig3          = figure(103);
fig1.WindowState = 'maximized';
fig2.Position = [2600 20 560 420];
% fig3.Position = [3300 20 560 420];

for k = 1:K

    % true state
    t                   = dt * k;
    x1_t(:,k)           = true_state(t, r, w, alpha);

    % IMU sensor data 
    [wb(k), fb(:,k), wb_n(k), fb_n(:,k), gy, ac] = simulate_IMU(r, w, dt);
    G1(:,k)             = gy.';  % grab noise for later analysis
    A1(:,k)             = ac(1,:).';
    A2(:,k)             = ac(2,:).';

    % apply corrections (if applicable)
    if use_err_CL
%         [wb_err_hat, fb_err_hat] = error_estimator('get', dt*(k-k_GPS));
        [wb_err_hat, fb_err_hat, gy, ac] = error_estimator('get', dt*(k-k_GPS));
        wb_c(k)         = wb(k)   - wb_err_hat;
        fb_c(:,k)       = fb(:,k) - fb_err_hat;

        Ge1(:,k)             = gy.';  % grab error est. later analysis
        Ae1(:,k)             = ac(1,:).';
        Ae2(:,k)             = ac(2,:).';
    else
        wb_c(k)         = wb(k);
        fb_c(:,k)       = fb(:,k);
    end
    
    % inertial navigation
    x1_tilde(:,k)       = inertial_navigation(fb_c(:,k), wb_c(k), dt);

    % Kalman Filter predict (also when dx=0, because we want to track P)
    [F, G]              = motion_model(x1_tilde(:,k), fb_c(:,k), betas);
    [dx, P]             = KalmanFilter('predict', F, G, Q, dt*(k-k_GPS));
    
    % GNSS update
    if ~rem(k, round(f/f_GPS))
        k_GPS           = k;                % keep track of time since last GPS update
        kk              = ceil(k*f_GPS/f);  % index of the current GPS update

        % GNSS sensor data
        z_GPS(:,kk)     = simulate_GNSS(x1_t(:,k));

        % Differential Kalman Filter update
        dz_GPS          = z_GPS(:,kk)-x1_tilde(4:5,k);
        [dx, P]         = KalmanFilter('update', dz_GPS, H, R);
    end

    % apply error corrections 
    x1_hat(:,k)         = x1_tilde(:,k) + dx(1:5);
    sigma_p_hat(:,k)    = sigma_p(P);

    if use_EKF && (k == k_GPS) 
        inertial_navigation('set', x1_hat(:,k));
        KalmanFilter('init', zeros(size(dx)), P);

        if use_err_CL 
            error_estimator('update', dx(6:end), betas, dt_GPS);
        end
    end

    % outputs for report
%     if (k == k_GPS) && any(kk-1 == [1, 10])
%         fprintf("for GNSS update %d:\n", kk-1)
%         fprintf("\tBefore: expected accuracy is %5.2f m\n", sigma_p_hat(k-1))
%         fprintf("\tBefore: error is             %5.2f m\n", vecnorm(x1_hat(4:5,k-1)-x1_t(4:5,k-1)))
% 
%         fprintf("\tAfter:  expected accuracy is %5.2f m\n", sigma_p_hat(k))
%         fprintf("\tAfter:  error is             %5.2f m\n", vecnorm(x1_hat(4:5,k)-x1_t(4:5,k)))
%     end


%     if ~rem(k, round(f/4))
    if ~rem(k, round(200*f))
        fig1 = plot_everything({trajectory, x1_t(4:5,k), x1_hat(4:5,k), x1_hat(4:5,1:k), P(4:5,4:5), z_GPS(:, 1:kk)}, ...
                               {dt*(1:k), fb_n(:,1:k), rad2deg(wb_n(1:k)), ...
                                          fb(:,1:k),   rad2deg(wb(1:k)),   ...
                                          fb_c(:,1:k), rad2deg(wb_c(1:k))},...
                                fig1, vis_mode);
        sgtitle(simtitle)

        figure(fig2), clf
        colororder([1, 0, 0;  0, 0.5, 0;  0, 0.5, 0])
        stb_k = round(60*f);
        h = []; l = [];
        if ~use_EKF
            yyaxis left % prediction
            plot(dt*(1:k), vecnorm(x1_tilde(4:5,1:k)-x1_t(4:5,1:k)))
            h = [h, ylabel("$\widetilde{x}_1$ error [m]")];
            l = [l, {'$\Delta \tilde{x}_1$'}];
            yyaxis right % estimation + uncertainty
        end
            plot(dt*(1:k), vecnorm(x1_hat(4:5,1:k)-x1_t(4:5,1:k))), hold on
            plot(dt*(1:k), sigma_p_hat(1:k), '--');
            l = [l, {'$\Delta \hat{x}_1$', '$\hat{\sigma}_p$'}];
            if k > stb_k
                plot(dt*[stb_k,k], sqrt(sum(var(x1_hat(4:5,stb_k:k)-x1_t(4:5,stb_k:k), [], 2)))*[1,1], '--', 'LineWidth', 1)
                l = [l, '$\sigma_p$'];
                plot(dt*[stb_k, stb_k], [0, max(sigma_p_hat(1:k))], 'k--')
                text(dt*stb_k + 1, max(sigma_p_hat(1:k))/2,  ["steady","state"])
            end
            h = [h, ylabel("$\widehat{x}_1$ error [m]")];
        axis tight
        grid on 
        xlabel("time [s]")
        h = [h, legend(l)];
        set(h,'interpreter','Latex');
        title(errtitle)

        drawnow
    end

end

%%


figure()
sgtitle("noise components")
subplot(3,1,1)
    plot(dt*(1:K), A1)
    ylabel("Ac1 noise [m/s^2]")
%     sigmas = std(A1, [], 2) / g / sqrt(f) * 1e6;
%     fprintf("sigma_acc1_wn = %5.1f [ug/sqrt(Hz)]\n" + ...
%             "sigma_acc1_GM = %5.1f [ug/sqrt(Hz)]\n", sigmas([1,3]));
subplot(3,1,2)
    plot(dt*(1:K), A2)
    ylabel("Ac2 noise [m/s^2]")
%     sigmas = std(A2, [], 2) / g / sqrt(f) * 1e6;
%     fprintf("sigma_acc2_wn = %5.1f [ug/sqrt(Hz)]\n" + ...
%             "sigma_acc2_GM = %5.1f [ug/sqrt(Hz)]\n", sigmas([1,3]));
subplot(3,1,3)
    plot(dt*(1:K), G1)
    ylabel("Gyro noise [rad/s]")
%     sigmas = rad2deg(std(G1, [], 2));
%     sigmas(1) = sigmas(1) * sqrt(60^2 /f);
%     sigmas(3) = sigmas(3) / sqrt(f);
%     fprintf("sigma_gyro_wn = %5.3f [deg/sqrt(h)]\n" + ...
%             "sigma_gyro_GM = %5.3f [deg/s/sqrt(Hz)]\n", sigmas([1,3]));

figure()
sgtitle("estimates")
subplot(3,1,1)
    plot(dt*(1:k), -Ae1)
    ylabel("Ac1 bias [m/s^2]")
subplot(3,1,2)
    plot(dt*(1:k), -Ae2)
    ylabel("Ac2 bais [m/s^2]")
subplot(3,1,3)
    plot(dt*(1:k), -Ge1)
    ylabel("Gyro bias [rad/s]")

figure()
sgtitle("Comparison GM bias : real vs. estimate")
colororder([1, 0, 0; 0, 0.5, 0])
subplot(3,1,1)
    plot(dt*(1:k), [A1(3,:); -Ae1(3,:)])
    ylabel("Ac1 b_{GM} [m/s^2]")
    legend(["real", "estimate"])
subplot(3,1,2)
    plot(dt*(1:k), [A2(3,:); -Ae2(3,:)])
    ylabel("Ac2 b_{GM} [m/s^2]")
subplot(3,1,3)
    plot(dt*(1:k), [G1(3,:); -Ge1(3,:)])
    ylabel("Gyro b_{GM} [rad/s]")

% figure()
% plot(dt*(1:k), exp(-betas.*dt.*(1:k)))

%%
disp("done")