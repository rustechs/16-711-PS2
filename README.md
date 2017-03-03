# 16-711: Assignment 2
Dynamics simulation and closed-loop control of a cart-pole system.

Authors: Thu Nguyen & Alexander Volkov

Note: This software requires a MuJoCo license to build and run!

## Files
- **model/symbolic-model/cartPoleSymbolic.m** - MATLAB script to generate cart-pole EoM's symbolically. Also linearizes system and computes LQR control gains.
- **model/cart-pole.xml** - The cart-pole MuJoCo model definition
- **source/cart-pole-sim.cpp** - main source file, contains simulator setup and controller implementations
- **source/makefile** - makefile for project; run 'make' in source dir to build
- **include/*** - Header files required by MuJoCo simulator
- **bin/cart-pole** - The cart-pole executable

## Overview
In this assignment, we play with dynamics and feedback control. In particular, we focus on the underactuated cart-pole system, first deriving its equations of motion symbolically, then simulating the system in a general purpose dynamics simulation environment (MuJoCo). In addition to simulating the passive dynamics, various feedback control methods were implemented in simulation in order to investigate the ability to stabilize the system about the vertical pole position (a naturally unstable equilibrium point). 

## Results
### Part 1: Symbolic Dynamics

The equations of motion of the cart-pole system were derived using the Lagrangian formulation of dynamics, carried out through the MATLAB Symbolic Toolbox. The input to the symbolic solver was simply 

TODO

### Part 2: Simulated Dynamics

TODO

### Part 3: Linear Controller, Manual Gain Tuning

TODO

### Part 4: LQR Controller

TODO

### Parts 5 & 6: Nonlinear Control for Swing Up Maneuver

TODO
