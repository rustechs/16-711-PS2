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
syms x(t) xDot(t) theta(t) thetaDot(t) f(t) tau(t) q(t) qDot(t) L(x,xDot,q,qDot,t)

tau = [f 0];

% Generalized coord's
xDot = diff(x(t),t);
thetaDot = diff(theta(t),t);
q = [x theta]';
qDot = [xDot thetaDot]';

% System Energies
KE = 0.5*(Mc + Mp)*xDot^2 + 0.5*(Ip + Mp*(l/2)^2)*thetaDot^2;
PE = Mp*g*l*cos(theta)/2;

% Compute lagrangian
L = KE-PE;

% Create symbolic equation of motion
dLdqDot = gradient(L(t),qDot);
dLdq = gradient(L,q);

% Equate generalized torques (RHS) with LHS terms
tau = diff(dLdqDot,t) - dLdq;