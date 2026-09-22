# Experiment Logic vs Platform Integration (Stage 9)

Two layers, one clean seam. **Experiment-specific algorithms never go inside the
platform adapter.**

```
┌─ EXPERIMENT LOGIC ──────────────────────────────────────────────┐
│  package: as2_experiment_aerpaw_multiuav                          │
│  · researcher's algorithm            · decision making            │
│  · wireless / RF / network metrics   · optimisation               │
│  · mission objectives                · multi-UAV coordination      │
│  talks to vehicles ONLY via AeroStack2 (as2_python_api / topics)   │
└───────────────────────────────┬─────────────────────────────────┘
                                 │  AS2 public API (the seam)
┌───────────────────────────────▼─────────────────────────────────┐
│  PLATFORM INTEGRATION  (package: as2_platform_aerpaw)            │
│  · AERPAW ↔ AeroStack2 communication (UDP-JSON IpcBridge)         │
│  · state translation      · command translation                   │
│  · platform abstraction (as2::AerialPlatform)                    │
│  NO mission / RF / decision / coordination logic here             │
└──────────────────────────────────────────────────────────────────┘
```

## 1. Platform integration — owns only

| Responsibility | Where |
|---|---|
| AERPAW↔AS2 communication | `ipc_bridge`, `aerpawlib_runner/aerpaw_as2_runner.py` |
| State translation | `state_translation.{hpp,cpp}` + `frame_conversions.hpp` |
| Command translation | `command_translation.{hpp,cpp}`, `command_capability.hpp` |
| Platform abstraction | `AerpawPlatform : as2::AerialPlatform` (`aerpaw_platform.{hpp,cpp}`) |
| Vehicle id / multi-UAV *mapping* (mechanism, not policy) | `vehicle_identity.hpp` |

The adapter moves **one** vehicle per instance. It has no idea what a "mission" or
a "formation" or an "RF metric" is, and must not gain such knowledge.

## 2. Experiment logic — owns only

| Responsibility | Where |
|---|---|
| Researcher algorithm / objectives / decision / optimisation | `as2_experiment_aerpaw_multiuav/experiment/decision.py` |
| Multi-UAV coordination (roles, formation geometry) | `experiment/coordination.py` |
| Wireless / RF / network metrics | `experiment/rf_metrics.py` |
| The seam to the platform | `experiment/interfaces.py` (`PlatformInterface`) |
| Concrete missions | `missions/*.py` |

## 3. The seam (contract between the layers)

Experiment code depends on `PlatformInterface`, implemented by `DroneInterfacePlatform`
over `as2_python_api.DroneInterface`. The only things that cross the seam are
**standard AeroStack2 constructs**:

- Down (command): AS2 behaviours / motion references → `actuator_command/*`,
  `platform` services, control-mode negotiation.
- Up (state): `sensor_measurements/*` → `self_localization/*`, `platform_info`,
  plus experiment-level `RFMetrics` from the AERPAW RF side (a `MeasurementSource`).

**Wireless measurement seam (Stage 10):** AERPAW RF/network metrics are delivered by
the adapter **as opaque transport** onto a standard ROS topic
`<ns>/aerpaw/measurements` (`std_msgs/String`, JSON). The experiment layer reads it
through `experiment.rf_metrics.TopicMeasurementSource` (or any `MeasurementSource`)
and feeds `closed_loop.ClosedLoopRunner` + an injected `RobotPolicy`. The adapter never
interprets a metric and no research algorithm lives there; because the topic is a
standard `std_msgs` type, the experiment still does not import any adapter symbol.

**Forbidden across the seam (enforced):** experiment code importing `ipc_bridge`,
`aerpaw_platform`, `aerpaw_as2_runner`, `aerpawlib`, `as2_platform_aerpaw`, or touching
the UDP IPC ports; and the adapter importing the experiment package. See
`test/test_layering.py`.

## 4. Adding a new experiment

A researcher writes a `PlatformInterface`-driven mission + a `decision`/`coordination`
policy + an optional `rf_metrics.MeasurementSource`. They **never touch**
`as2_platform_aerpaw`. Conversely, fixing/adding a vehicle capability happens in the
adapter only, with no experiment code changing.

## 5. Enforcement

`python3 as2_experiment_aerpaw_multiuav/test/test_layering.py` (also run by
`colcon test`). It strips docstrings/comments and flags only real import/usage, so the
boundary is checked in CI, not just documented.
