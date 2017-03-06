# 16-711: Assignment 2
Dynamics simulation and closed-loop control of a cart-pole system.

Authors: Thu Nguyen & Alexander Volkov

Note: This software requires a MuJoCo license to build and run!

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

[Cart-Pole Passive Dynamics](https://youtu.be/VUy_G41N8dA)

Notably, this model includes only two rigid bodies (cart and pole), and two unconstrained degrees of freedom (one translational of the cart relative to the inertial frame, one rotational of the pole attached to the cart).

### Part 3: Linear Controller, Manual Gain Tuning

The above passive dynamics model of the cart-pole was modified to include an arbitrary controllable force along the cart's axis of motion. No saturation limits or additional dynamics were included on this actuation input.

A linear controller of the form `u = -Kz` was implemented in order to stabilize the pole about the upright position. `z = [x theta xDot thetaDot]` is the state vector of the system, and `K = [k1 k2 k3 k4] = [-300,30000,-600,1000]` is the gain matrix (vector). Note, this linear controller stabilizes the full nonlinear system about its passively unstable equilibrium point at the state-space origin `z = 0`, but with a limited region of attraction. Small deviations from the origin are stabilized succesfully, but larger initial errors result in the system going unstable. This is because the linear controller only guarantees global stability for the linearized cart-pole system, which is a reasonable approximation only for small errors about the origin. At larger initial errors, the system's nonlinearities deviate too significantly from the linearized model, leading the controller to act erroneously (or insufficiently).

A video of the system stabilizing using the hand-tuned linear controller is shown below:

[Cart-Pole Linear Controller (Manually Tuned Gains) Stabilization](https://youtu.be/8Z9-TPiBseM)

### Part 4: LQR Controller

As noted above, the manually-tuned linear controller is able to stabilize the cart-pole system about the state-space origin, but with a rather limited region of attraction. One option is to use a more intelligent method of choosing the linear gains `K = [k1 k2 k3 k4]` in order to increase the range of initial conditions that can be stabilized. An LQR controller is a good option, given its ubiquity. In order to design the LQR controller, the equations of motion derived previously were linearized symbollically about the state-space origin, resulting in the following linear system:

```
dx = Ax + Bu

A =
         0         0    1.0000         0
         0         0         0    1.0000
         0    0.9810         0         0
         0   21.5820         0         0
         
B =
         0
         0
    0.0010
    0.0020

```

The LQR gains were computed using MATLAB's `lqr` function, with the following Q,R matrices:

```
Q = diag([1 4 0.1 0.1]);
R = 0.00001;
```

The following video shows the LQR controller in action, stabilizing the cart-pole system about the state-space origin. Note that the initial error in pole angle in the following example is greater than the maximum stabilizable angle by the origin linear controller -- that is, the LQR controller has a larger region of attraction! 

[Cart-Pole LQR Stabilization](https://youtu.be/AylGBCjTifg)

### Parts 5 & 6: Nonlinear Control for Swing Up Maneuver

These two parts of the assignment were accomplished simultaneously with a globally asymtotically stable nonlinear controller, effectively capable of automatically performing the appropriate swing-up maneuver.

The particular control architecture used is an energy-shaping controller on the pole, with partial feedback linearization in order to cancel the cart dynamics, and an LQR controller to asymtotically stabilize the pole once it approaches the near-vertical configuration. Additionally, a PD controller is used to drive the cart to the desired state. This architecture is taken from Chapter 3.5 of Russ Tedrake's "Underactuated Robotics": http://underactuated.csail.mit.edu/underactuated.html?chapter=3.

The following snippet of code from `source/cart-pole-sim.cpp` implements the controller described above:

```
// K linear gain vector corresponding to [x theta xDot thetaDot]
mjtNum K[4] = {-300,30000,-600,1000};
mjtNum Klqr[4] = {-316,25452,-977,5519};
mjtNum kNL[3] = {0.5,10,10};
mjtNum nlOutputClamp = 1000;

// Control output
mjtNum u;

mjtNum theta = 0;

if (d->qpos[1] > 0) {
    theta = fmod(d->qpos[1]+mjPI,2*mjPI) - mjPI;
} else {
    theta = fmod(d->qpos[1]-mjPI,2*mjPI) + mjPI;
}

mjtNum p = 0;
mjtNum Eerr = 0;

// mjtNum absErr = mju_sqrt(mju_pow(theta,2)+mju_pow(d->qvel[1],2));

// printf("Absolute Error: %f\n",absErr);

// LQR once near state-space origin
if (mju_abs(theta) < mjPI/4 && (mju_abs(d->qvel[1]) < mjPI && theta*(d->qvel[1]) >= 0 || mju_abs(d->qvel[1]) < mjPI && theta*(d->qvel[1]) < 0)) {
    printf("Switched to LQR!\n");

    u = -(K[0]*d->qpos[0] + K[1]*theta + K[2]*d->qvel[0] + K[3]*d->qvel[1]);
// Start with energy shaping controller
} else {

    printf("Switched to Energy Shaping Control!\n");

    Eerr = 12.5*mju_pow(d->qvel[1],2) + 490.5*mju_cos(theta) - 490.5;

    p = -mju_tan(theta)*4.905 - kNL[0]*d->qvel[1]*mju_cos(theta)*Eerr - kNL[1]*(d->qpos[0]) - kNL[2]*(d->qvel[0]);

    if (p < -nlOutputClamp) {
        p = -nlOutputClamp;
    } else if (p > nlOutputClamp) {
        p = nlOutputClamp;
    }   

    u = (1100 - 100*mju_pow(mju_cos(theta),2))*p-981*mju_sin(theta)*mju_cos(theta) + 50*mju_pow(d->qvel[1],2)*mju_sin(theta);

}

printf("Control Output: %f\n",u);
printf("State: [%f,%f,%f,%f]\n\n",d->qpos[0],theta,d->qvel[0],d->qvel[1]);

mju_copy(d->ctrl,&u,1);    
```
[Cart-Pole Swing-up using Energy Shaping Controller](https://www.youtube.com/watch?v=YL3aPpFbIhg)
