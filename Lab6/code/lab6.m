addpath("functions")
addpath("data")

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

% read datafile
name = "IXSEA";
% [data, f] = readimu("1106_1508_PostProBinaryDecoded.imu", name);
[data, f] = readimu("1112_AirINS_Decoded.imu", name);
disp("-")

% extract assigned range: 
% t_range = [488485	488600];    % range originally assigned
% t_range = [488585	488615];    % low-noise (extended) original range
% t_range = [488310	488430];    % range 2, assigned by TA due to bad data
                                % in original range 
t_range = [488390	488430];    % range 2, low-noise part of new range

tidx    = (data(:,1) > min(t_range)) & (data(:,1) < max(t_range));
t       = data(tidx,1) - 488000;
gyro    = data(tidx, 2:4);
accel   = data(tidx, 5:end);


% reference attitude ( in NWD ) 
% r       = deg2rad( 10.298); % originally assigned range
% p       = deg2rad(  0.120);
% y       = deg2rad(319.710); 

r       = deg2rad( 15.039); % range 2
p       = deg2rad( -0.063);		
y       = deg2rad(  7.537); 

% change reference attitude to NED
p       = -p;

% true earth rotation 
we      = 7.2921150e-5; % [rad/s]

% true earth gravity in NED
g       = (980000+550) * 1e-5; % [m/s^2]

% reference latitude
lat     = deg2rad( 46 + 31/60 + 17/60^2 ) ;

disp("Note: disregard all +/- accuracy estimations below, they are wrong!")


%%----------------------------------------------------------
%% Task 1: plot the data
%%----------------------------------------------------------

if plt
    disp('<strong>Task 1:</strong>');
    disp("plotting")
end


if plt 
    % do linear regression to show non-constant accelerations
    X = [ones(size(t)), t];
    Ba = X \ accel;

    fig = figure(1);
    fig.Name = "Accelerometer Data";
    ylbls = {'f_x [m s^{-2}]', 'f_y [m s^{-2}]', 'f_z [m s^{-2}]'};
    for i = 1:3
        subplot(3,1,i)
            plot(t, accel(:,i)), hold on
            plot(t, Ba(1,i) + t*Ba(2,i), 'r-', 'linewidth', 1.5)
            xlabel("GPS time [s], epoch 488000"); ylabel(ylbls{i})
            grid on
            axis('tight')
    end
    legend(["data", "trendline"])
    sgtitle(sprintf("%s accelerometer data", name))
end 

if plt 
    % do linear regression to show non-constant rates
    X = [ones(size(t)), t];
    Bg = X \ gyro;

    fig = figure(2);
    fig.Name = "Gyroscope Data";
    ylbls = {'\omega_x [rad s^{-1}]', '\omega_y [rad s^{-1}]', '\omega_z [rad s^{-1}]'};
    for i = 1:3
        subplot(3,1,i)
            plot(t, gyro(:,i)), hold on
            plot(t, Bg(1,i) + t*Bg(2,i), 'r-', 'linewidth', 1.5)
            xlabel("GPS time [s], epoch 488000"); ylabel(ylbls{i})
            grid on
            axis('tight')
    end
    legend(["data", "trendline"])
    sgtitle(sprintf("%s gyroscope data", name))
end 


%%----------------------------------------------------------
%% Task 2: find the norm of the gyro data 
%%----------------------------------------------------------

% estimated earth rotation from gyro data
we_bar    = mean(vecnorm(gyro,  2, 2), 1);

% estimate gyro accuracy
gyro_bias = abs(we-we_bar);

disp('<strong>Task 2:</strong>');
fprintf("An w_E of %.3e rad/s was observed. "+...
        "The true value is %.3e rad/s.\n", ...
        we_bar, we);
fprintf("\tThe gyros have an accuracy of %.3g rad/s.\n", gyro_bias)


%%----------------------------------------------------------
%% Task 3: find the norm of the accelerometer data 
%%----------------------------------------------------------


% eastimated earth gravity from gyros in NED
g_bar      = mean(vecnorm(accel, 2, 2), 1);

% estimate accelerometer accuracy 
accel_bias = abs(g-g_bar);

disp('<strong>Task 3:</strong>');
fprintf("A g of %.3g m/s^2 was observed. "+...
        "The true value is %.3g m/s^2.\n", ...
        g_bar, g);
fprintf("\tThe accelerometers have an accuracy of %.3g m/s^2.\n", accel_bias)


%%----------------------------------------------------------
%% Task 4: perform accelerometer levelling 
%%----------------------------------------------------------

% chose mean values to work from & transform to NED
fb       = RotMat(pi, 1) * mean(accel,1).';

% perform levelling to NED
r_bar    = atan2(-fb(2) , -fb(3));
r_err    = accel_bias/g;
p_bar    = atan2( fb(1) , sqrt(fb(3)^2+fb(2)^2));
p_err    = accel_bias/g;


disp('<strong>Task 4:</strong>');
fprintf("The calculated roll  is %7.3f° +/- %.1e°. " + ...
        "The reference is %7.3f°.\n", ...
        rad2deg(r_bar), rad2deg(r_err), rad2deg(r));
fprintf("The calculated pitch is %7.3f° +/- %.1e°. " + ...
        "The reference is %7.3f°.\n", ...
        rad2deg(p_bar), rad2deg(p_err), rad2deg(p));

fprintf("\tThe error is %4.1g° roll & %4.1g° pitch.\n", rad2deg(r-r_bar), rad2deg(p-p_bar));


%%----------------------------------------------------------
%% Task 5: perform gyrocompassing 
%%----------------------------------------------------------


% chose mean values to work from & transform to NED
wb       = RotMat(pi, 1) * mean(gyro,1).';

% perform gyrocompassing in NED-level frame
R_level  = RotMat(-p_bar, 2) * RotMat(-r_bar, 1);
w_level  = R_level * wb;
y_bar    = wrapTo2Pi(atan2(-w_level(2), w_level(1)));
y_err    = gyro_bias / (we*cos(lat));   % okay, this is cheating because we don't know the latitude yet.

disp('<strong>Task 5:</strong>');
fprintf("The calculated yaw   is %7.3f° +/- %.1e°. " + ...
        "The reference is %7.3f°.\n", ...
        rad2deg(y_bar), rad2deg(y_err), rad2deg(y));
fprintf("\tThe error is %.2g° yaw.\n", rad2deg(y-y_bar));


% estimate latitude from NED level frame
latbar = atan(-w_level(3) / sqrt(w_level(1)^2 + w_level(2)^2) ); 
% latbar = asin(-w_level(3) / we ); % in our weird case, this works better?
laterr = gyro_bias/we;

fprintf("The estimated latitude is N %d° %d' %.0f"" +/- %5.3f°. " + ...
        "True latitude is N %d° %d' %.0f"". \n", ...
        rad2dms(latbar), rad2deg(laterr), rad2dms(lat));
fprintf("\tThis is off by %d° %d' %.0f"". \n", ...
        rad2dms(lat - latbar))



%%
disp("-")
disp("done")