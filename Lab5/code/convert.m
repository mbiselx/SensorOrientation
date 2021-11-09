clc

f               = 100;                        % [Hz]
fprintf("for a sampling frequency of %g [Hz]:\n", f)

%%%%%%
disp("------")

gyr_cb_sigma    = deg2rad(10/3600);           % [rad/s] constant bias std()
fprintf("gyr_cb_sigma is %.3g [rad/s]\n", gyr_cb_sigma)

gyr_GM_corrT    = 100;                        % [s]     correlation time of GMP
gyr_GM_sigma    = deg2rad(5e-3);              % [rad /s /sqrt(Hz)]
gyr_GM_sqrt_qk  = gyr_GM_sigma*sqrt(f*(1-exp(-2*dt/gyr_GM_corrT)));  % [rad/s/sample] GM driving sequence std()
fprintf("gyr_GM_sigma is %.3g [rad/s/sqrt(Hz)], and gyr_GM_corrT is %.3g [s]\n", gyr_GM_sigma, gyr_GM_corrT)
fprintf("\t this gives a gyr_GM_sqrt_qk of %.3g [rad/s/sample]\n", gyr_GM_sqrt_qk)

gyr_wn_sigma    = deg2rad(1e-1)*sqrt(f/3600); % [rad/s/sample] white noise std()
fprintf("sigma_g_wn is %.3g [rad/s/sample]\n", gyr_wn_sigma)

%%%%%%
disp("------")

acc_cb_sigma  = 9.81 *  1e-3;                 % [m/s^2] constant bias std()
fprintf("acc_cb_sigma is %.3g [m/s^2]\n", acc_cb_sigma)

acc_wn_sigma  = 9.81 * 50e-6 * sqrt(f);       % [m/s^2/sample] white noise std()
fprintf("acc_wn_sigma is %.3g [m/s^2/sample]\n", acc_wn_sigma)



%%%%%%
disp("Done")