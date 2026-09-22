# AERPAW + AeroStack2 — Documentation Index

Bridge that lets any AeroStack2 (AS2) mission run unchanged on the **AERPAW Digital
Twin / SITL**, in addition to the AS2 **Multirotor Simulator** and **Gazebo** backends.

## Pick the right document

| I want to… | Read |
|------------|------|
| Know what to install / requirements | [`run_and_test_requirements.txt`](run_and_test_requirements.txt) |
| Understand the whole workflow end‑to‑end and run it | [`E2E_MANUAL.md`](E2E_MANUAL.md) |
| Learn how the AERPAW↔AS2 bridge works (reference) | [`as2_platform_aerpaw` README](../as2_aerial_platforms/as2_platform_aerpaw/README.md) |
| Run the example 3‑drone mission / see the config commands | [`as2_experiment_aerpaw_multiuav` README](../as2_experiment_aerpaw_multiuav/README.md) |

Single source of truth per topic: **requirements** = what you need · **E2E** = the how‑to
· **platform README** = bridge reference · **experiment README** = mission + run commands.

## Repo map

```
docs/
  README.md                       this index
  run_and_test_requirements.txt   prerequisites (levels 1–3)
  E2E_MANUAL.md                   end-to-end guide

as2_aerial_platforms/as2_platform_aerpaw/     the platform bridge
  src/ + include/                C++ node + UDP-JSON IPC bridge
  aerpawlib_runner/              Python aerpawlib runner
  config/  launch/  tests/       params, launch files, gtests
  README.md                      bridge reference

as2_experiment_aerpaw_multiuav/               the example mission
  missions/triangle_formation.py 3-drone formation
  launch/                         Config A / B / C launch files
  README.md                      mission + run commands
```

> Also at repo root: [`../aerpaw_as2_quickstart.html`](../aerpaw_as2_quickstart.html) — the
> minimal **style template** these docs follow (not a manual) · [`PLAN.md`](../PLAN.md)
> (original task spec) · [`HOST.md`](../HOST.md) (dev host access).

## Status

- **Build + unit tests (Level 1):** pass on ROS 2 Humble.
- **Config A** (Multirotor Simulator): flies the 3‑drone formation.
- **Config B** (Gazebo): runs on Gazebo **Harmonic / ROS 2 Jazzy** (Humble's Fortress
  lacks the platform control‑mode support — see the experiment README).
- **Config C** (AERPAW DT / SITL): SITL endpoints validated; SPEED/velocity mode is
  limited by aerpawlib (see the platform README *Limitations*).
