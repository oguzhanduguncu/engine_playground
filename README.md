# engine_playground
---
oguzhan learns RT systems and engine subsystems
---
# Engine Playground

A research-oriented C++ engine playground focused on real-time simulation,
deterministic physics, and time ownership.

This project explores how professional simulation and RTS engines structure
their update loops, handle fixed timesteps, and avoid instability issues such
as tunneling and nondeterminism.

Most modern game engines are frame-driven.
This works well for visual responsiveness but introduces subtle instability
and nondeterminism in physics-heavy or simulation-oriented systems.

This project exists to explore an alternative approach:

- Physics owns time
- Simulation runs at a fixed timestep
- Rendering interpolates instead of driving logic
- Audio lives in a separate real-time domain

## Engine Time Model

The engine is structured around a single principle:

> Time is produced by the engine core, consumed by simulation,
> and merely observed by rendering.

Subsystems:
- Physics: fixed timestep, authoritative
- Render: variable FPS, interpolated
- Audio: callback-driven, real-time safe

## Fixed vs Variable Timestep

Using variable delta time for physics leads to:
- Frame-rate dependent behavior
- Unstable collision response
- Tunneling under high velocities

This engine uses a fixed timestep accumulator model to ensure:
- Stable simulation
- Predictable collision handling
- Deterministic replay potential

## Collision Detection and Tunneling

Discrete collision checks can miss fast-moving objects,
a phenomenon known as tunneling.

This project demonstrates:
- Why reducing dt is not scalable
- How collision time (TOI) can be predicted
- How CCD can be integrated without breaking the fixed timestep model

## What This Project Is Not

- Not a game
- Not a full-featured engine
- Not optimized for production

It is an experimental lab designed to understand
engine-level design decisions.

---

## What I Learned

- Why physics should not be frame-driven
- How engines avoid the spiral of death
- How real-time systems differ from visual systems
- Why determinism must be designed from day one
