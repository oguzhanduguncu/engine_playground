
| System      | Rhythm Type      | Feature               |
| ----------- | --------------- | --------------------- |
| Engine Core | Monotonic       | Real owner of the time|
| Physics     | Fixed timestep  | Deterministic         |
| Audio       | Callback driven | Hard RT               |
| Render      | Variable        | Visual output         |


# Engine Core / Time Ownership

## Motivation
To demonstrate different usage type of systems in an engine. How engine orchestrates all of these system ? Also, to recognize different type of systems with time models as fixed timestep, hard RT and pipeline.
## Who owns time?
Engine core owns the time and it never depends or driven by any subsytem.
## System rhythms
This is the most important topic to synthesize the harmony between the systems:
Audio is the most strict one. Every buffers must be completed before audio callback. Otherwise, there will be a glitch.
Physics uses the fixed timestep to provide consistent behaviour for whole simulation.
Render operates on a variable timestep driven by display refresh rate and GPU scheduling. It is the most tolerant system in terms of timing deviations.
In real-time engines, most timing-related engineering effort is spent on audio and physics, since rendering is generally tolerant to temporal inconsistencies.
## Fixed vs Variable timestep
Fixed timestep means the simulation advances in constant time steps independent of render framerate. For the physics system this model **MUST** be followed. Imagine a system changes the simulation rules due to system hardwares.
For the variable timestep, like "dt" changes with the frequency of the renderer:
dt varies for 60 FPS rendering and 144 FPS rendering.
Then, integration calculations change:
for the basic mass-spring simulation:
F = -k*x
a = F/m
v = a*dt
x = v*dt
For the variable timestep tunneling, body instabilities and lots of subtle may occur.
Also, by the wider timestep, possibility of the instabilies may increase.
Physics waits for none of the other systems. It just produces for evety timesteps.
Mathematically,
For 60 FPS physics system(simulation), if we have 60 FPS renderer, there is no problem. They fit.
For a 100 FPS simulation, if we have 50 FPS renderer, for a one frame system simulates two step.
For a 100 FPS simulation, if we have 60 FPS renderer, simulator continues to produce in the fixed timestep, but renderer shows 1 or sometimes 2 step. An accumulator can be used there.
accumulator += frame_time

while accumulator >= dt:
simulate(dt)
accumulator -= dt

For a 50 FPS simulation, if we have 60 FPS renderer, there would be a visual stutter. Because, for the most of the frames accumulator < dt. This creates a time alias and it looks like a micro-stutter.
Interpolation can solve it:

alpha = accumulator / dt
render_state = lerp(prev_state, curr_state, alpha)

Continuous Collision Detection (CCD) does not simulate continuously.
Instead, it analytically computes the time of impact between objects over a fixed timestep by sweeping volumes along their trajectories. CCD allows collision detection to be independent of timestep size, as long as motion can be approximated linearly over dt. Therefore, it does not miss any collision and does not tunnel. It checks time discrete timesteps again, but for the fixed timestep calculations are linear, thus collision time is computable. For the computed time intervals, volume will be calculated then intersection can be found for the time of the hit.

## Audio as a separate time domain
Audio is the hard real time system of this engine. Audio runs in a hard real-time domain driven by the audio device clock, not by the engine loop.(The engine can only communicate with the audio system through lock-free, non-blocking control paths). Because, it produces a buffer and it will be used instantly. Everyting it uses must be computable. There is no kernel call, no 3rd party not-RT library and no locks. It just uses an already allocated memory.
## Thread model
Audio runs on the different thread than other systems. Because, for the real time systems, we do not want any loss of time like cache miss or an unbounded operation.Audio is isolated on its own thread to guarantee bounded execution time and avoid interference from cache misses, locks, or unbounded operations in other systems.

## What this lab does NOT do
This is just an experimental LAB to see the different type of systems. It is NOT a product.
## What I learned
I learned how different subsystems operate under fundamentally different time constraints and how an engine core must coordinate them without forcing a single frame-based timeline.
Sample working frequencies:
Physics tick: 60 Hz \
Render submit: 120 Hz \
Audio callback: 48kHz \
ASCII timeflow:
```text
No drift detected

Time →        |----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|
Index         |    0     |    1     |    2     |    3     |    4     |    5     |    6     |    7     |    8     |    9     |   10     |

Engine Core:  |----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|
Index         |    0     |    1     |    2     |    3     |    4     |    5     |    6     |    7     |    8     |    9     |   10     |

Physics:      |---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
Index         |   0     |   1     |   2     |   3     |   4     |   5     |   6     |   7     |   8     |   9     |  10     |  11     |  12     |

Render:       |----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|
Index         | 0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  | 9  |10  |11  |12  |13  |14  |15  |16  |17  |18  |19  |20  |21  |22  |23  |24  |25  |

Audio:        | . . . . | . . . . | . . . . | . . . . | . . . . | . . . . | . . . . | . . . . | . . . . | . . . . | . . . . |
Index         |    0    |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
              48 kHz sample ticks (buffered callbacks)
```


- Audio does not depend on engine core
- Physics needs accumulator
- render needs interpolation