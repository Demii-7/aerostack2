# AERPAW + AeroStack2 — Documentation Index

Bridge that lets any AeroStack2 (AS2) mission run unchanged on the **AERPAW Digital
Twin / SITL**, in addition to the AS2 **Multirotor Simulator** and **Gazebo** backends.

## Pick the right document

| I want to… | Read |
|------------|------|
| Know what to install / requirements | [`run_and_test_requirements.txt`](run_and_test_requirements.txt) |
| Understand the whole workflow end‑to‑end and run it | [`E2E_MANUAL.md`](E2E_MANUAL.md) |
| Understand the **current architecture** (as‑is, gaps, next components) | [`ARCHITECTURE.md`](ARCHITECTURE.md) |
| Know the **coordinate frames / units / conventions** & where conversions live | [`COORDINATE_FRAMES.md`](COORDINATE_FRAMES.md) |
| See which **AeroStack2 behaviors/controllers run on AERPAW** unchanged | [`AS2_CAPABILITIES.md`](AS2_CAPABILITIES.md) |
| Understand the **experiment-logic vs platform-integration** boundary | [`LAYERING.md`](LAYERING.md) |
| See the **explicit, versioned interfaces** (AERPAW↔Adapter↔AS2↔Experiment) | [`INTERFACES.md`](INTERFACES.md) |
| Learn how the AERPAW↔AS2 bridge works (reference) | [`as2_platform_aerpaw` README](../as2_aerial_platforms/as2_platform_aerpaw/README.md) |
| Run the example 3‑drone mission / see the config commands | [`as2_experiment_aerpaw_multiuav` README](../as2_experiment_aerpaw_multiuav/README.md) |
| Bring up the **standalone AERPAW emulator** and run its demo (no AS2) | [`AERPAW_EMULATOR.md`](AERPAW_EMULATOR.md) |

Single source of truth per topic: **requirements** = what you need · **E2E** = the how‑to
· **platform README** = bridge reference · **experiment README** = mission + run commands.

## Repo map

```
docs/
  README.md                       this index
  run_and_test_requirements.txt   prerequisites (levels 1–3)
  E2E_MANUAL.md                   end-to-end guide
  ARCHITECTURE.md                 as-is architecture + ownership (§6)
  COORDINATE_FRAMES.md            frame/unit conventions + conversion index
  INTERFACES.md                   explicit versioned interfaces (IF-1/2/3)
  AS2_CAPABILITIES.md             which AS2 behaviors run on AERPAW
  LAYERING.md                     experiment-logic vs platform-integration boundary
  AERPAW_EMULATOR.md              standalone emulator stack + demo (no AS2)

as2_aerial_platforms/as2_platform_aerpaw/     the platform bridge
  src/ + include/                C++ node + UDP-JSON IPC bridge
  aerpawlib_runner/              Python aerpawlib runner (AERPAW-side)
  launch/as2_stack.launch.py     AS2 robotics subsystem (AERPAW starts this)
  deploy/                        cvm_setup.sh · qgc_tunnel.sh · run_aerpaw_experiment.sh
  config/  launch/  tests/       params, dev-harness launch files, gtests
  README.md                      bridge reference

as2_experiment_aerpaw_multiuav/               the example mission
  missions/triangle_formation.py 3-drone formation
  launch/                         Config A / B / C launch files (dev harness)
  README.md                      mission + run commands
```

> **Ownership (Stage 2): AERPAW is the top-level experiment platform.** AERPAW
> owns the experiment lifecycle/config, Digital Twin, physical testbed, RF,
> resources, deployment and data collection. AeroStack2 is a **robotics subsystem**
> that AERPAW starts — AS2 never launches or manages AERPAW. Production bring-up is
> `deploy/run_aerpaw_experiment.sh`; details in [`ARCHITECTURE.md`](ARCHITECTURE.md) §6.

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
