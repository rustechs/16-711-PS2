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

The equations of motion of the cart-pole system were derived using the Lagrangian formulation of dynamics, carried out through the MATLAB Symbolic Toolbox. The inputs to the symbolic solver were the kinetic and potential energy terms of the cart-pole system (manually derived), vector of generalized forces, and the general Lagrange Equation:

```
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

% Compute Lagrange equations
LHS = jacobian(dLdqDot,[q qDot])*[qDot qDDot]'-dLdq;
```
The resulting equations of motion were thus:

```
(Mp*l*sin(theta)*thetaDot^2)/2 + xDDot*(Mc + Mp) - (Mp*l*thetaDDot*cos(theta))/2 = f
 Ip*thetaDDot + (Mp*l^2*thetaDDot)/4 - (Mp*g*l*sin(theta))/2 - (Mp*l*xDDot*cos(theta))/2 = 0
``` 
This matches the expected result for this ubiquitously studied system, readily available online.

### Part 2: Simulated Dynamics

The cart-pole dynamics were simulated and visualized using MuJoCo, a general purpose dynamics engine. The simulated passive dynamics (unactuated / uncontrolled) are illustrated in the following video:


Notably, this model includes only two rigid bodies (cart and pole), and two unconstrained degrees of freedom (one translational of the cart relative to the inertial frame, one rotational of the pole attached to the cart).

### Part 3: Linear Controller, Manual Gain Tuning

The above passive dynamics model of the cart-pole was modified to include an arbitrary controllable force along the cart's axis of motion. No saturation limits or additional dynamics were included on this actuation input.

A linear controller of the form

### Part 4: LQR Controller

TODO

### Parts 5 & 6: Nonlinear Control for Swing Up Maneuver
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

The equations of motion of the cart-pole system were derived using the Lagrangian formulation of dynamics, carried out through the MATLAB Symbolic Toolbox. The inputs to the symbolic solver were the kinetic and potential energy terms of the cart-pole system (manually derived), vector of generalized forces, and the general Lagrange Equation:

```
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

% Compute Lagrange equations
LHS = jacobian(dLdqDot,[q qDot])*[qDot qDDot]'-dLdq;
```
The resulting equations of motion were thus:

```
(Mp*l*sin(theta)*thetaDot^2)/2 + xDDot*(Mc + Mp) - (Mp*l*thetaDDot*cos(theta))/2 = f
 Ip*thetaDDot + (Mp*l^2*thetaDDot)/4 - (Mp*g*l*sin(theta))/2 - (Mp*l*xDDot*cos(theta))/2 = 0
``` 
This matches the expected result for this ubiquitously studied system, readily available online.

### Part 2: Simulated Dynamics

The cart-pole dynamics were simulated and visualized using MuJoCo, a general purpose dynamics engine. The simulated passive dynamics (unactuated / uncontrolled) are illustrated in the following video:

[TODO: passive dynamics video]

Notably, this model includes only two rigid bodies (cart and pole), and two unconstrained degrees of freedom (one translational of the cart relative to the inertial frame, one rotational of the pole attached to the cart).

### Part 3: Linear Controller, Manual Gain Tuning

The above passive dynamics model of the cart-pole was modified to include an arbitrary controllable force along the cart's axis of motion. No saturation limits or additional dynamics were included on this actuation input.

A linear controller of the form `u = -Kz` was implemented in order to stabilize the pole about the upright position. `z = [x theta xDot thetaDot]` is the state vector of the system, and `K = [k1 k2 k3 k4] = [-300,30000,-600,1000]` is the gain matrix (vector). Note, this linear controller stabilizes the full nonlinear system about its passively unstable equilibrium point at the state-space origin `z = 0`, but with a limited region of attraction. Small deviations from the origin are stabilized succesfully, but larger initial errors result in the system going unstable. This is because the linear controller only guarantees global stability for the linearized cart-pole system, which is a reasonable approximation only for small errors about the origin. At larger initial errors, the system's nonlinearities deviate too significantly from the linearized model, leading the controller to act erroneously (or insufficiently).

A video of the system stabilizing using the hand-tuned linear controller is shown below:

[TODO: manual linear gains video]

### Part 4: LQR Controller

As noted above, the manually-tuned linear controller is able to stabilize the cart-pole system about the state-space origin, but with a rather limited region of attraction. One option is to use a more intelligent method of choosing the linear gains `K = [k1 k2 k3 k4]` in order to increase the range of initial conditions that can be stabilized. An LQR controller is a good option, given its ubiquity. [TODO]

[TODO: video of LQR controller]

### Parts 5 & 6: Nonlinear Control for Swing Up Maneuver

[Implement nonlinear controller]

[Video of nonlinear controller in action]
