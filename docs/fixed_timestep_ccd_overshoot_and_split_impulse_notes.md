# Fixed Timestep CCD, Overshoot, and Split Impulse
- The overshoot observed at the moment of impact is a consequence of committing the simulation state only at fixed timestep boundaries
- CCD guarantees correct impact timing but does not eliminate transient positional error within a discrete integration step.
## Scenario (2D Physics)

```
C(t)
 ^
 |                         /
 |                        /
 |                       /
 |                      /
 |                     /
 |--------------------•---------------------> t
 |                   /  t0
 |                  /
 |                 /
 |                /
 |               /
 |______________/
 |
```

- **Contact normal**: \( n = (-1, 0) \)
- **Dynamic object position**: \( x_A = 10.1 \)
- **Static object position**: \( x_B = 10 \)

In a **fixed timestep** engine, even with effective **CCD**, this situation is mathematically common. During contact, additional dynamics are required to keep positions valid.

There are two major approaches:
- **Position correction**
- **Position-Based Dynamics (PBD)**

Position correction can be **force-based** or **impulse-based**. In this engine, **split impulse** is used.

---

## Contact Constraint Definition

- Contact normal: \( n \)
- Penetration depth: \( d ≥ 0 \)

- C(x) = n · (xA − xB) ≥ 0

### Calculation for the Scenario

C(x) = -1 · (0.1) = -0.1

This means the constraint is **violated**.

---

## Constraint Solvers

There are two solver levels:

- **Velocity-level solver**
- **Position-level solver**

### Velocity-Level Solver (Idea)

dC/dt >= 0

This ensures that the constraint is **not getting worse** after the contact.

However:
- C(t_0) can still be **negative**.
- The violation already exists and must be corrected.

Therefore, a **position-level correction** is required.

---

### Position-Level Solver (Idea)

- x' = x + Δx
- C(x') ≥ 0

Assume correction along the normal:

Δx = λ * n


#### a) Geometric Solution

λ = d

- Direct projection onto the surface
- Causes **jitter**

#### b) Timestep-Based Solution (Linearized)

Δx ≈ v_pseudo * dt

v_pseudo = Δx / dt

With bias and slop:

Δx = β * max(d − slop, 0) * n


v_pseudo = (β / dt) * max(d − slop, 0) * n

---

## Pseudo Impulse (Split Impulse)

J_pseudo = v_pseudo / invMass

Properties:
- **Not applied to real velocity**
- Only corrects **position**
- Hence the name **split impulse**

---

## Engine Logs at Collision Time

```
x=9.91667 vx=9.91667 ax=5
y=11.9    vy=10.9167 ay=5
wall=10 dt=0.0166667

TOI.hit=1 TOI.t=0.00838563
DISCRETE HIT: x=10.0828 wall=10 pen=0.0828103
AFTER STEP manifolds=1
RENDER manifolds.size = 1
```

Next frame:

```
x=10 vx=0 ax=5
y=12.083 vy=6 ay=5
wall=10 dt=0.0166667

DISCRETE HIT: x=10.0014 wall=10 pen=0.00138889
AFTER STEP manifolds=1
RENDER manifolds.size = 1
```

Observation:
- There is still **overshoot at the first collision moment**.

---

## Why Overshoot Still Happens

Timeline:

### \( t_n \)
- Integrate → **TOI**
- Apply impulse → \( dC/dt ≥ 0 \)
- Integrate **remaining time**

### \( t_{n+1} \)
- Position correction

Key point:
- The engine is **fixed timestep**.
- Collision resolution is a **state update at the end of the timestep**.
- The first overshoot happens **before** position correction is applied.

---

## Tunneling vs User Experience

Mitigations:

- **Slop threshold**
  - Ignores tiny instantaneous penetrations

- **Physics faster than rendering** or **interpolation**
  - Physics: \( dt = 1/60 \) or faster
  - Render: \( dt = 1/60 \)

---

## How to Completely Remove Overshoot

### 1) Sub-Stepping

Fixes overshoot at step-0:

```cpp
remaining = dt - t_hit;

while (remaining > epsilon) {
    integrate(small_step);
    solve_contacts();
    remaining -= small_step;
}
```

**Disadvantages**:
- Higher CPU cost
- Harder determinism

**Used by**:
- Bullet
- Havok

---

### 2) Continuous Position Correction

Impulse moment: x(t_hit+) = projection onto constraint surface

Meaning:

- C(t_hit+) = 0
- dC/dt ≥ 0

**Disadvantages**:
- Breaks velocity–position consistency
- Energy explosion risk

**Used by**:
- Robotics
- Offline physics simulations

---

### 3) Speculative Contacts

- Collision is activated **before** it actually happens
- Impulse applied early

**Disadvantages**:
- Complex manifold management

**Used by**:
- Box2D

