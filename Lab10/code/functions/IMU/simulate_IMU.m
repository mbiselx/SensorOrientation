function [wb, fb, wb_n, fb_n, a, b] = simulate_IMU(r, w, dt)
%SIMULATE_IMU Summary of this function goes here
%   Detailed explanation goes here

    wb_n           = w;                   % nominal signal
    fb_n           = [0; r*w^2];
%     [wb_err, fb_err] = simulate_IMU_noise('get', dt);
    [wb_err, fb_err, a, b] = simulate_IMU_noise('get', dt);
    wb             = wb_n + wb_err;       % apply noise
    fb             = fb_n + fb_err;

end

