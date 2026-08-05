# Model notes

## Active coordinates

The package uses the front-left leg sign convention from the upstream controller:

```text
q = [q_hip, q_sweep, q_extend]
```

- `q_hip` rotates around the proximal x axis and represents the inward/outward hip motion.
- `q_sweep` rotates the complete planar leg mechanism.
- `q_extend` drives one short link of the four-bar mechanism.

The latter does not behave like an independent serial-knee joint. It changes two passive angles through the closed-loop geometry and therefore changes the effective leg length.

## Four-bar closure

The passive-angle calculation is adapted from:

```text
quadruped_controller/src/leg.cpp
Leg::update_passive_joints(double q3)
```

The link lengths are taken from:

```text
quadruped_controller/include/quadruped_controller/leg.hpp
UPPER_BONE_LENGTH = 0.125 m
LOWER_BONE_LENGTH = 0.210 m
```

The active-joint axes and offsets are taken from:

```text
four_bar_bot_description/urdf/leg.urdf.xacro
```

## Kinematic MPC

The demonstration uses the discrete-time model

```text
q[k+1] = q[k] + dt * u[k]
```

where `u[k]` contains the three active joint velocities. The finite-horizon objective penalizes:

1. foot-position tracking error;
2. deviation from a nominal recoverable posture;
3. joint velocity;
4. changes in joint velocity.

The nonlinear constraints enforce joint limits, a lateral-spread margin, and a four-bar toggle margin.

## Interpretation of safety margins

The margins are explicit and inspectable, but they are still heuristics. They are not yet hardware-validated Control Barrier Functions. A research-grade safety layer should connect each margin to:

- geometric reachability and branch selection;
- actuator torque reserve;
- friction and contact state;
- recoverability under bounded model error;
- real-time solver feasibility.

## Next model levels

A sensible progression is:

1. kinematic one-leg MPC, as implemented here;
2. torque-level one-leg dynamics and parameter identification;
3. contact/friction constraints and barrier functions;
4. a centroidal or Single Rigid Body model for the quadruped;
5. a Whole-Body Control layer mapping body/contact plans to joint commands;
6. integration with the existing reinforcement-learning policy as a proposal layer or residual controller.
