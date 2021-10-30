% paternoster
clc 
close all
clear 

addpath('functions')

%% simulate 2D circular motion
% the mapping frame is defined as a NED frame
% the body frame is defined with the x axis along the direction of motion 

f     = 50;     % [Hz]      sampling frequency
int_method = 'Trapezoid'; %   integration method 

R     = 500;    % [m]       radius of the trajectory 
w     = pi/100; % [rad/s]   angular speed
alpha = pi/2;   % [rad]     initial heading
trajectory = R*[cos(2*pi*(0:1/50:1)); sin(2*pi*(0:1/50:1))];

r     = R;      % [m]       radial position
dr    = 0;      % [m/s]     change of radial position
ddr   = 0;      % [m/s^2]   radial acceleration
psi   = 0;      % [rad]     angular direction in map
dpsi  = w;      % [rad/s]   rotation rate
ddpsi = 0;      % [rad/s^2] change in rotation rate
dt    = 1/f;    % [s]       sampling period 


% initialize the inertial navigation function with position, velocity, 
% acceleration and attitude estimations, as well as integration method
x0    = [     R;   0];
v0    = [     0; R*w];
a0    = [-R*w^2;   0];
azth0 = alpha;
inertial_navigation('init', x0, v0, a0, azth0, int_method);


% prepare simulation loop
T = 2*pi/w;

% allocate memory 
wb       = zeros(size(dt:dt:T));
fb       = zeros(2, length(wb));
x_err    = zeros(size(wb));
v_err    = zeros(size(wb));
azth_err = zeros(size(wb));

% simulation loop
k = 1;
for t = dt:dt:T
    % use polar cordinates to generate the true position, velocity and
    % heading
    x_t         =   R*[ cos(w*t);  sin(w*t)];
    v_t         = w*R*[-sin(w*t);  cos(w*t)];
    azth_t      = alpha + w*t;
    % use polar cordinates to generate the sensor data 
    fb(:,k)     = [2*dr*dpsi + r*ddpsi;
                   -(ddr - r*dpsi^2)];
    wb(k)       = dpsi;

    % estimate the current position from IMU data
    [x_est, v_est, ~, azth_est] = inertial_navigation(fb(:,k), wb(k), dt);

    % calculate the error in the estimation
    delta_x     = x_est - x_t;
    delta_v     = v_est - v_t;
    x_err(k)    = sqrt(delta_x.'*delta_x);
    v_err(k)    = sqrt(delta_v.'*delta_v);
    azth_err(k) = mod(azth_est - azth_t + pi, 2*pi) - pi; % make sure it's in [-pi:pi]

    % do a fun plot
% %     if toc > 5e-3
%     if ~rem(t,10*dt)
%         fig = plot_everything({trajectory, x_t, x_est}, {dt:dt:t, fb(1, 1:k), fb(2, 1:k), rad2deg(wb(1:k))});
% %         tic;
% 
% %         Capture the plot as an image 
%         [imind,cm] = rgb2ind(frame2im(getframe(fig)),256); 
% %         Write to the GIF File 
%         filename = 'test.gif';
%         if t == 0 
%           imwrite(imind,cm,filename, 'gif', 'Loopcount',inf, 'DelayTime', .05); 
%         else 
%           imwrite(imind,cm,filename, 'gif','WriteMode','append', 'DelayTime', .05); 
%         end 
%     end

    k = k+1;
end 

%% plot the errors 
fprintf("The final error is [%g, %g] [m].\n", delta_x(1), delta_x(2))

figure
subplot(3,1,1)
    plot(dt:dt:T, x_err)
    grid on
    xlabel('t [s]'), ylabel('err_{x} [m]')
    legend("error in position")
subplot(3,1,2)
    plot(dt:dt:T, v_err)
    grid on
    xlabel('t [s]'), ylabel('err_{v} [m/s]')
    legend("error in velocity")
subplot(3,1,3)
    plot(dt:dt:T, rad2deg(azth_err))
    grid on
    xlabel('t [s]'), ylabel('err_{\alpha} [°]')
%     set(gca,'yticklabel',num2str(get(gca,'ytick')','%g'))
    legend("error in heading")

sgtitle(sprintf("Errors for f = %d Hz and %s integration method", f, int_method))

%%
disp("done")