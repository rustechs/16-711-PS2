%% Swing-up control for cart-pole system using Energy Shaping

% clean up
clc;clear;close all

% Compute system dynamics symbolically
cartPoleSymbolic;

LHSstd = simplify(subs(LHSstd,{Mc,Mp,Ip,g,l},[1000,100,0,9.81,1]));
xDDot = simplify(subs(xDDot,{Mc,Mp,Ip,g,l},[1000,100,0,9.81,1]));
thetaDDot = simplify(subs(thetaDDot,{Mc,Mp,Ip,g,l},[1000,100,0,9.81,1]));