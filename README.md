# engine_playground
oguzhan learns RT systems and engine subsystems

| System      | Type of Time    | Feature               |
| ----------- | --------------- | --------------------- |
| Engine Core | Monotonic       | Real owner of the time|
| Physics     | Fixed timestep  | Deterministic         |
| Audio       | Callback driven | Hard RT               |
| Render      | Variable        | Visual output         |

Time Hierarchy:

┌─────────────────────────────┐
│       ENGINE CORE           │ Engine core never depends or driven by any subsystem!
│   (monotonic clock)         │
├─────────────────────────────┤
│   SYSTEM SCHEDULING         │ 
│   (tick / callback)         │
├─────────────────────────────┤
│   SUBSYSTEMS                │
│   Physics / Audio / Render  │
└─────────────────────────────┘


# LAB 1 — Engine Core / Time Ownership

## Motivation
  To demonstrate different usage type of systems in an engine. How engine orchestrates all of these system ? Also, to recognize different type of systems with time models as fixed timestep, hard RT and pipeline.
## Who owns time?
  Engine core owns the time and it never depends or driven by any subsytem.
## System rhythms
## Fixed vs Variable timestep
## Audio as a separate time domain
## Thread model
## Common pitfalls
## What this lab does NOT do
## What I learned

Sample working frequencies:
Physics tick: 60 Hz
Render submit: ~144 Hz
Audio callback: 48kHz
No drift detected

ASCII timeflow:
Time →

Engine Core:

Physics:

Render:

Audio:
