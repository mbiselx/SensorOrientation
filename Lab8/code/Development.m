clc
close all 
clear

addpath("functions\")

%%
syms q [2,1] 
syms r [2,1] 
syms delta_t real
syms x0 [6,1] real
syms p0 [6,1] real 
syms z [2,1] real

P0 = diag(p0);

Phat = P0;
xhat = x0;

% time derivative system
F  = diag(ones(1,4), 2);
H = [eye(2), zeros(2,4)];
G = [zeros(4,2); eye(2)];

% discretize
Phi = expm(F * delta_t);

% system noise
Q = diag(q);
Qk = int(Phi*G*Q*G.'*Phi.', delta_t, 0, delta_t);

% measurement noise 
R = diag(r);

% Kalman filtering loop
N = 2;
disp("starting the training run")
for i = 1:N
    % predictor
    xtilde = Phi * xhat;
    Ptilde = Phi * Phat * Phi.' + Qk;
    
    % gain
    K = simplify( (Ptilde * H.') / (H * Ptilde * H.' + R) );
    
    % state update 
    xhat = xtilde + K * (z-H*xtilde);
    
    % covariance update 
    Phat = (eye(size(K,1), size(H,2)) - K*H)*Ptilde;
end
disp("finished the training run")

%% now do the same with the KalmanFilter function

% Initialize
KalmanFilter("init", x0, P0);

% Kalman filtering loop
disp("starting the testing run")
for i=1:N
    [xtilde_KF, Ptilde_KF]  = KalmanFilter("predict", F, G, Q, delta_t);
    [xhat_KF, Phat_KF]      = KalmanFilter("update", z, H, R);
end
disp("finished the testing run")

% verify results 
assert(~any(simplify(xtilde_KF - xtilde), "all"), "bad xtilde?");
assert(~any(simplify(Ptilde_KF - Ptilde), "all"), "bad Ptilde?");

assert(~any(simplify(xhat_KF - xhat), "all"), "bad xhat?");
assert(~any(simplify(Phat_KF - Phat), "all"), "bad Phat?");

