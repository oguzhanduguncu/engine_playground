# Collision, Tunneling & CCD Motivation

## Motivation

This lab demonstrates a fundamental limitation of **discrete collision detection**
when combined with a **fixed timestep simulation**.

While a fixed timestep guarantees determinism and numerical stability,
it does **not** guarantee correct collision detection for fast-moving objects.
This phenomenon is known as **tunneling**.

The goal of this lab is not to fix tunneling immediately,
but to **reproduce it intentionally** and understand *why* it happens.

---

## Experimental Setup

- One dynamic object moving along the x-axis
- One static, infinitely thin wall located at `x = 10`
- Fixed timestep physics simulation (`dt = 1 / 60`)
- Discrete collision detection based on position sampling

Collision condition (naive discrete check):

```
if (x_old < wall_x && x_new >= wall_x)
    collision detected
```

---

## Discrete Collision Failure (Tunneling)

In this setup, the object may move from one side of the wall to the other
within a single simulation step without ever being observed *at* the wall position.

As a result:
- The collision is detected **after** the object has already crossed the wall
- The logged collision position may be greater than the actual wall location
  (e.g. `x = 10.0688` while the wall is at `x = 10.0`)
- At this point, the collision information is no longer physically meaningful

The object is effectively treated as if it passed through the wall.

This behavior is called **tunneling**.

---

## Why Fixed Timestep Is Not Enough

Reducing the timestep (`dt`) makes tunneling *less likely* by increasing the
temporal sampling resolution.
However, this is **not a real solution**:

- Smaller `dt` increases CPU cost
- It reduces scalability
- It increases the risk of spiral-of-death scenarios
- It still does not provide a mathematical guarantee against tunneling

A fixed timestep ensures **deterministic updates**,
but it does **not** ensure correct collision timing.

---

## Root Cause

The core issue is not the collision algorithm itself,
but the **discrete sampling of time**.

Discrete collision detection answers the question:

> “Has a collision already happened between two snapshots?”

It does **not** answer:

> “When exactly did the collision occur?”

This distinction is critical.

---

## Continuous Collision Detection (CCD Motivation)

To resolve tunneling without reducing the timestep,
the collision must be evaluated in **continuous time**.

Instead of checking only the endpoints of a timestep,
CCD computes the **time of impact (TOI)** within the interval `[0, dt]`
by solving for the exact moment when the moving object intersects the wall.

This allows the simulation to:
- Detect collisions reliably
- Advance the state *precisely* to the collision moment
- Preserve determinism without increasing simulation frequency

---

## What This Lab Does NOT Do

- No full rigid-body solver
- No rotation or angular dynamics
- No broad-phase collision optimization
- No production-grade CCD implementation

This lab is intentionally minimal and focuses on **conceptual correctness**.

---

## What I Learned

- Fixed timestep improves determinism, not collision accuracy
- Discrete collision detection can miss fast interactions
- Tunneling is a time-sampling problem, not a collision bug
- CCD exists to compute *when* a collision happens, not just *that* it happened

