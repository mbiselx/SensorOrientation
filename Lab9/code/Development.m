clc
close all 
clear

addpath("functions\")

%% variables
syms Q [2,1] 
syms R [2,1] 
syms delta_t real
syms x0 [3,1] real
syms P0 [3,1] real 
syms z [2,1] real

P0 = diag(P0);
Q  = diag(Q);
R  = diag(R);

Phat = P0;
xhat = x0;

%% system

F       = zeros(3); F(2,3) = 1;
G       = [1 0 ; 0 0 ; 0 1];

h       = @(x) (x(1) * [cos(x(2)); ...
                        sin(x(2))]);
evalH   = @(x) [cos(x(2))  -x(1)*sin(x(2))  0;
                sin(x(2))   x(1)*cos(x(2))  0];

% transformation to cartesian space [p; v] = m(x)
m       = @(x) ([  x(1) * cos(x(2)); ...
                   x(1) * sin(x(2)); ...
                 - x(1) * x(3) * sin(x(2)); ...
                   x(1) * x(3) * cos(x(2))]); 


%% discretize the system
[Phi, Qk] = discretize_model(F, G, Q, delta_t);

%% Kalman filtering loop
N = 0;
disp("starting the training run")
for i = 1:N
    % predictor
    xtilde = Phi * xhat;
    Ptilde = Phi * Phat * Phi.' + Qk;
    
    % gain
    H = evalH(xtilde);
    K = simplify( (Ptilde * H.') / (H * Ptilde * H.' + R) );
    
    % state update 
    xhat = xtilde + K * (z-h(xtilde));
    
    % covariance update 
    Phat = simplify(eye(size(K,1), size(H,2)) - K*H)*Ptilde;
end
disp("finished the training run")


