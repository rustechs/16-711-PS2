%% cartPoleSymbolic.m
%
%  Generate the EOMs of a simple planar cart-pole system symbolically.
%
%  Uses Lagrangian formulation of dynamics.
%
%%%

%% Clean up
clc; clear; close all

%% Lagrangian Dynamics
% Define symbolic parameters
syms Mc Mp Ip l t KE PE g real
% Define symbolic time-dependent state variables
syms x xDot xDDot theta thetaDot thetaDDot f tau q qDot L real

% Generalized forces
tau = [f 0]';

% Generalized coord's
q = [x theta];
qDot = [xDot thetaDot];
qDDot = [xDDot thetaDDot];

% System Energies
i = [1 0];
j = [0 1];
vp = (xDot - l/2*cos(theta)*thetaDot)*i + (-l/2*sin(theta)*thetaDot)*j;

KEc = 0.5*(Mc)*xDot^2;
KEp = 0.5*(Mp)*(vp*vp') + 0.5*Ip*thetaDot^2;
KE = KEc + KEp;
PE = Mp*g*l*cos(theta)/2;

% Compute lagrangian
L = KE-PE;

% Create symbolic equation of motion
dLdqDot = gradient(L,qDot);
dLdq = gradient(L,q);

% Compute Lagrange Equations
LHS = jacobian(dLdqDot,[q qDot])*[qDot qDDot]'-dLdq;

% simplify!
LHSstd = simplify(LHS);

% Convert to state space
LHS = LHSstd - tau;

[xDDot,thetaDDot] = solve(LHS(1),LHS(2),xDDot,thetaDDot);

z = [x theta xDot thetaDot];
zDot = [xDot thetaDot xDDot thetaDDot];
u = f;

% Use feedback linearization to stabilize system anywhere
zDot = simplify(zDot);

% Linearize system about z = 0, u = 0;
A = subs(jacobian(zDot,z),{Mc,Mp,Ip,g,l,x,xDot,theta,thetaDot},[1000,100,0,9.81,1,0,0,0,0]);
B = subs(jacobian(zDot,u),{Mc,Mp,Ip,g,l,x,xDot,theta,thetaDot},[1000,100,0,9.81,1,0,0,0,0]);

A = double(subs(simplify(A),{Mc,Mp,Ip,g,l,x,xDot,theta,thetaDot},[1000,100,0,9.81,1,0,0,0,0]));
B = double(subs(simplify(B),{Mc,Mp,Ip,g,l,x,xDot,theta,thetaDot},[1000,100,0,9.81,1,0,0,0,0]));

% Generate LQR controller
Q = diag([1 4 0.1 0.1]);
R = 0.00001;
[K,~,lambda] = lqr(A,B,Q,R,zeros(4,1));



