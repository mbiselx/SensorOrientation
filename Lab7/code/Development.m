clc
close all 
clear

%%
syms qvn qve rpn rpe
syms delta_t real
syms x0 [4,1] real
syms p0 [4,1] real 
syms z [2,1] real

P0 = diag(p0);

Phat = P0;
xhat = x0;

% time derivative system
F = [zeros(2), eye(2); zeros(2, 4)];
G = [zeros(2); eye(2)];
H = [eye(2), zeros(2)];

% discretize
Phi = expm(F * delta_t);

% system noise
Q = [ qvn, 0 ; 0, qve];
Qk = int(Phi*G*Q*G.'*Phi.', delta_t, 0, delta_t);

% measurement noise 
R = [ rpn, 0 ; 0, rpe];

% predictor
xtilde = Phi * xhat;
Ptilde = Phi * Phat * Phi.' + Qk;

% gain
K = simplify( Ptilde * H.' / (H*Ptilde*H.' + R) , 'steps', 50);

% state update 
xhat = xtilde + K * (z-H*xtilde);

% covariance update 
Phat = (eye(4) - K*H)*Ptilde;

% predictor
xtilde = simplify(Phi * xhat  , 'steps', 50);
Ptilde = Phi * Phat * Phi.' + Qk;
