# From Overshoot to Stable Contacts

## Building a Robust 1D Collision Pipeline (CCD → Manifold → Impulse → Stabilization)

This document describes the **step-by-step evolution** of a 1D rigid body collision system whose original problem was **overshoot** under fixed-timestep integration. The goal was not to hide the issue with hacks, but to **systematically build a physically correct pipeline**, starting from CCD and ending with stable contact resolution.

The result is a clean, extensible core that mirrors how real physics engines (Box2D / Bullet-style) structure collision handling.

---

## 1. The Original Problem: Overshoot in 1D

We start with a minimal simulation:

* Single dynamic body
* One static vertical wall at `x = wall.x`
* Fixed timestep integration
* Constant acceleration toward the wall

Using semi-implicit Euler integration:

```text
v += a * dt
x += v * dt
```

### Observed issue

At sufficiently high velocities or accelerations, the body **crosses the wall between timesteps**. This produces:

* Tunneling
* Large penetration
* Non-deterministic behavior

This is the classical **overshoot problem**.

---

## 2. Step 1 — Continuous Collision Detection (CCD)

### Motivation

Discrete collision tests are insufficient when motion between frames is large. We need to answer:

> *Did the body hit the wall at any time inside the timestep?*

### Approach

We introduce **Time of Impact (TOI)** computation in 1D.

For motion with acceleration:

```text
x(t) = x0 + v0 * t + 0.5 * a * t²
```

Solve for:

```text
x(t) = wall.x
```

If a valid solution `t ∈ [0, dt]` exists, a collision occurred during the timestep.

### Result

* Overshoot is detected
* Tunneling is prevented
* But collision is still treated as a **single event**

At this stage, collision handling is still incomplete.

---

## 3. Step 2 — Introducing Contact Manifolds

### Why CCD Alone Is Not Enough

CCD tells us *when* the first contact happened, but not:

* How long the contact lasts
* Whether the body is still in contact next frame

To reason about contact **as state**, not just an event, we introduce **Contact Manifolds**.

### Contact Model

Even in 1D, we model contact using a general structure:

```text
ContactManifold
  - bodyA, bodyB
  - pointCount
  - ContactPoint[]

ContactPoint
  - position
  - normal
  - penetration
```

This abstraction allows future extension to:

* Multiple contact points
* Friction
* Stacking

---

## 4. Step 3 — Discrete Contact Phase (Persistence)

### Problem After CCD

CCD detects the *first frame* of impact only. In subsequent frames:

* TOI no longer triggers
* The body is already near or inside the wall
* Contact disappears unless explicitly re-detected

### Solution: Discrete Contact Phase

After integration, we perform a **discrete proximity test**:

```text
if body.x >= wall.x - slop → contact
```

Key ideas:

* Introduce a small **contact slop** tolerance
* Treat contact as a *persistent state*

### Result

* Contact manifolds persist across frames
* Overshoot is no longer hidden
* Penetration becomes measurable and stable

At this point, the engine can *see* the problem clearly.

---

## 5. Step 4 — Impulse-Based Collision Response

### Velocity-Level Overshoot

With persistent contact, we observe:

* Penetration increases every frame
* Body keeps accelerating into the wall

The next step is to stop **velocity-level overshoot**.

### Normal Impulse Solver

We resolve contact using an impulse applied along the contact normal:

```text
vn = vrel.dot(n)

j = -(1 + restitution) * vn / invMass
v += j * invMass * n
```

For a static wall:

* Only the dynamic body is modified
* The normal velocity component is driven to zero

### Result

* The body no longer accelerates into the wall
* Dynamic overshoot is eliminated
* Penetration still exists (by design)

This separation is intentional.

---

## 6. Step 5 — Split Impulse (Position-Level Correction)

### Why Not Fix Position Directly?

Directly correcting position via impulses or Baumgarte terms can:

* Inject artificial energy
* Affect bounce and friction
* Destabilize stacks

### Split Impulse Concept

Split impulse introduces a **separate correction velocity** used only for position correction:

```text
pseudoVelocity += λ * n
position += pseudoVelocity * dt
```

Where:

```text
λ = penetration / (dt * invMass)
```

This correction:

* Does not affect real velocity
* Does not change energy
* Only removes penetration

### Result

* Penetration converges to a small tolerance band
* Contact becomes visually and numerically stable
* Overshoot is fully controlled

---

## 7. Baumgarte vs Split Impulse

Although Baumgarte stabilization can also reduce penetration, split impulse was chosen because:

* It preserves physical velocity
* It does not alter restitution behavior
* It matches modern rigid body engine design

Baumgarte remains a valid alternative, but split impulse provides a cleaner separation of concerns.

---

## 8. Final Pipeline Summary

The final fixed-timestep pipeline is:

```text
Fixed Step:
  1. Integrate motion
  2. Continuous Collision Detection (TOI)
  3. Generate contact manifold (CCD)
  4. Discrete contact phase (persistence)
  5. Normal impulse solver (velocity-level)
  6. Split impulse (position-level correction)
```

Each stage solves **one specific problem**, without masking others.

---

## 9. Key Takeaways

* Overshoot is not a single bug, but a **multi-layered problem**
* CCD solves *event detection*
* Manifolds model *contact state*
* Impulses solve *velocity correctness*
* Split impulse solves *geometric stability*

This structured approach leads to a robust and extensible rigid body collision core, even in the simplest 1D case.

---

## 10. Outlook

From this foundation, the engine can naturally evolve toward:

* Friction (tangential impulses)
* Multiple contact points
* Kinematic bodies
* 2D / shape-based collision

The critical lesson is that **correct ordering and separation of responsibilities** is what ultimately eliminates overshoot—not ad-hoc fixes.

---

## Appendix A — Why CCD Alone Is Not Enough

Continuous Collision Detection (CCD) is often misunderstood as a *complete* solution to tunneling and overshoot. In reality, CCD solves **only one layer** of the problem.

### What CCD Actually Solves

CCD answers a very specific question:

> *Did two objects intersect at any time within the current timestep?*

It does **not**:

* Maintain contact state across frames
* Prevent long-term penetration
* Resolve forces or impulses
* Stabilize resting contacts

In other words, CCD is an **event detector**, not a contact manager.

### The One-Frame Contact Problem

Without a discrete contact phase:

* CCD reports a hit in the impact frame
* In the next frame, the objects are already overlapping
* TOI no longer triggers
* Contact information disappears

This leads to the classic symptom:

> *"Collision is detected, but only for a single frame."*

### Why Manifolds Are Required

Contact manifolds transform collision handling from:

* *"Something happened"* (event)

to:

* *"Something is happening"* (state)

By persisting contact information, manifolds allow:

* Stable stacking
* Friction
* Position correction
* Multi-contact resolution

### The Full Picture

Only when CCD is combined with:

* Discrete contact detection
* Impulse-based resolution
* Position-level stabilization

can overshoot be considered **fully addressed**.

CCD is necessary—but never sufficient—on its own.

---

## Appendix B — References and Further Reading

The design choices and pipeline described in this document are aligned with established rigid body physics literature and real-world engines. The following references are strongly recommended:

1. **Erin Catto** — *Iterative Dynamics with Temporal Coherence*
   (GDC 2005–2015, Box2D design papers)

2. **Erin Catto** — *Box2D: A 2D Physics Engine for Games*
   Explains contact manifolds, impulse solvers, and split impulse stabilization.

3. **Baraff & Witkin** — *Physically Based Modeling: Principles and Practice*
   Foundational reference for rigid body dynamics and constraint-based simulation.

4. **David Baraff** — *Fast Contact Force Computation for Nonpenetrating Rigid Bodies*
   Introduces constraint-based contact resolution and stability issues.

5. **Christer Ericson** — *Real-Time Collision Detection*
   Comprehensive coverage of collision detection, CCD, and contact generation.

6. **Bullet Physics SDK Documentation**
   Practical reference for CCD, manifolds, impulse solvers, and split impulse.

7. **Müller et al.** — *Position Based Dynamics*
   Useful contrast between impulse-based solvers and position-level approaches.

These works collectively demonstrate why modern physics engines rely on **layered collision pipelines**, rather than single-step fixes, to robustly eliminate overshoot and instability.
