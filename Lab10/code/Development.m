clc
close all 
clear

addpath("functions\")

%% variables
syms Q1 Q2 [3,1]  real
syms R [2,1]  real
syms delta_t  real
syms x0 [5,1] real
syms P0 [5,1] real 
syms z  [2,1] real
syms fb [2,1] real
syms wb       real
syms beta [3,1] real

P0 = diag(P0);
Q  = diag([Q1; Q2]);
Q1 = diag(Q1);
R  = diag(R);

Phat = P0;
xhat = x0;

%% INS

syms alpha real
acc   = RotMat(-alpha)*fb;
vel   = acc * delta_t;
pos   = [0; 0];

%% system

F11          = zeros(5, 'sym');
F11(2:3,1)   = RotMat(-alpha)*[0,-1;1,0]*fb;
F11(4:5,2:3) = eye(2);

G11          = [RotMat(-alpha,1); zeros(2,3)];


%% discretize the system
[Phi, Qk] = discretize_model(F11, G11, Q1, delta_t)
% simplify(Phi)
% simplify(Qk)

%% augment 

F12 = [[1;zeros(4,1)], G11];
F21 = zeros(4, 5, 'sym');
F22 = diag([0; -beta]);

F = [F11, F12; F21, F22]

G12 = zeros(5,3, 'sym');
G21 = zeros(4,3, 'sym');
G22 = [zeros(1,3,'sym'); eye(3, 'sym')];
G = [G11, G12; G21, G22]

% [Phi, Qk] = discretize_model(F, G, Q, delta_t);
% simplify(Phi)
% simplify(Qk)
