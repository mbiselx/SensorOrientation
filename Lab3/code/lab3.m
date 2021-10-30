% paternoster
clc
clear 
close all

%% Step 1: set up the R_el matrix, using the North, East and Down vectors
syms  phi  lam 

% North, East and Down vectors
d = [-cos(phi)*cos(lam);
     -cos(phi)*sin(lam);
     -sin(phi)];
n = - diff(d, phi);
e = - diff(d, lam)./cos(phi);

% rotation matrix 
R = [n, e, d];


%% Step 2: express North, East and Down time derivatives as functions  
syms dphi dlam

% North, East and Down vectors derivatives
dn = -e*sin(phi)*dlam + d*dphi; 
de =  (n*sin(phi) + d*cos(phi)) * dlam;
dd = -e*cos(phi)*dlam - n*dphi; 

% rotation matrix derivative
dR = simplify(diff(R, phi)*dphi + diff(R, lam)*dlam);

% check the solution
assert(all(double(simplify(dR == [dn, de, dd])), 'all'), "The proposed formulation is incorrect")
disp("The proposed formulation of dR is correct")


%% Step 3: 

Omega = simplify(R.' * dR)
assert(all(double(simplify(Omega == R \ dR)), 'all'), "The proposed calculation is incorrect")
disp("The proposed calculation of Omega is correct")

omega = [Omega(3,2);
         Omega(1,3);
         Omega(2,1)]

















disp("done")