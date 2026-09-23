# Standalone AERPAW Emulator — Run the Demo (no AeroStack2)

How to bring up the emulated AERPAW vehicle stack on the Chameleon host and run its
built-in `demo_square` experiment **without** AeroStack2 — pure AERPAW emulation
(ArduPilot SITL + MAVLink Router + aerpawlib E-VM containers).

The project and topology files live in the `teaching-on-testbeds/aerpaw-emulator`
checkout on the host (`/home/opencode/aerpaw-emulator`). All commands below run **on
the host**; from the opencode container prefix them with the ssh invocation from
[`HOST.md`](../HOST.md):

```bash
ssh -i /home/jovyan/.ssh/id_ed25519 -o BatchMode=yes -o IdentitiesOnly=yes \
    -o UpdateHostKeys=no opencode@host.docker.internal '<command>'
```

## 1. Start the 3-UAV stack

```bash
cd /home/opencode/aerpaw-emulator
docker compose -f docker-compose.three-uav.yml up -d          # image build is cached
docker compose -f docker-compose.three-uav.yml ps
```

Expected: 7 containers, the 3 `C-VM-*` and `MAVLink-Gateway` `(healthy)`.

What each part is:

| Service | Role |
|---|---|
| `nodeN-uav-cvm` (`C-VM-X0842-MN`) | ArduCopter SITL (MAVLink sysid N) + mavlink-router |
| `nodeN-uav-evm` (`E-VM-X0842-MN`) | aerpawlib + AERPAW profile; `run-aerpawlib` wrapper |
| `mavlink-gateway` | aggregates all 3 vehicles onto host `127.0.0.1:5760` |

Every E-VM consumes its own vehicle only (link isolation), on `udp:0.0.0.0:14550`
(vehicle data) and `udp:0.0.0.0:14551` (auto-arm).

## 2. Run the demo on one UAV

```bash
docker compose -f docker-compose.three-uav.yml exec node1-uav-evm run-aerpawlib demo_square
```

Expected output: `vehicle armed` → `Mission took 00:36` → `vehicle disarmed`.
The wrapper (`run-aerpawlib`) arms the SITL in `STABILIZE`, runs the script, lands,
and disarms.

## 3. Run it on all three

```bash
for n in 1 2 3; do
  docker compose -f docker-compose.three-uav.yml exec node$n-uav-evm run-aerpawlib demo_square
done
```

## 4. Tune the demo

```bash
docker compose -f docker-compose.three-uav.yml exec \
  -e SQUARE_SIZE=20 -e FLIGHT_ALT=10 node1-uav-evm run-aerpawlib demo_square
```

(`SQUARE_SIZE` m, default 10; `FLIGHT_ALT` m, default 5 — AERPAW testbed requires ≥ 20 m.)

## 5. Persist output to results

```bash
docker compose -f docker-compose.three-uav.yml exec -d node1-uav-evm sh -c \
  'run-aerpawlib demo_square > /results/demo.log 2>&1'
cat /home/opencode/aerpaw-emulator/results/842/node-1/demo.log   # on the host
```

## 6. QGroundControl via the gateway

All vehicles are on one MAVLink TCP link at host `127.0.0.1:5760` (or tunnel it):

```bash
ssh -N -L 5760:127.0.0.1:5760 user@remote-host
```

Add a **TCP link** to `localhost:5760`; each vehicle appears by its system ID (1/2/3).
The hub keeps each vehicle's traffic isolated and drops GCS heartbeats from the links.

## 7. Stop / restart

```bash
docker compose -f docker-compose.three-uav.yml stop    # keep containers
docker compose -f docker-compose.three-uav.yml start
docker compose -f docker-compose.three-uav.yml down    # remove; results/ is preserved
```

## 8. Other topologies & experiments

Only one stack at a time (every gateway publishes host `127.0.0.1:5760`):

| Compose file | Nodes | Built-in demo |
|---|---|---|
| `docker-compose.uav.yml` | 1 UAV | `demo_square` (no ZMQ) |
| `docker-compose.uav-ugv.yml` | UAV + UGV | `demo_square` + ZMQ coordinated-square |
| `docker-compose.two-uav.yml` | 2 UAVs | ZMQ tracer/orbiter |
| `docker-compose.three-uav.yml` | 3 UAVs | `demo_square` (this stack) |

Custom experiments: drop `<name>.py` in `experiments/`, then
`run-aerpawlib <name> --args...` (module args pass through). Set `AUTO_ARM=0` to skip
the wrapper's arm/disarm, or `VEHICLE_TYPE=none` for ZMQ coordinators.

Render a new topology (unique node ids 1–255, unique `experiment` number):

```bash
python3 tools/render_topology.py topology.my-nodes.yaml --output docker-compose.my-nodes.yml
docker compose -f docker-compose.my-nodes.yml up -d --remove-orphans
```

## 9. Troubleshooting

- C-VM not healthy → `docker logs C-VM-X0842-M1`; SITL crash usually means the
  parameter/home string is wrong.
- E-VM "Waiting for safety pilot to arm" forever → auto-arm failed on :14551; check
  the C-VM is healthy and retry.
- Gateway has no vehicles → `docker logs MAVLink-Gateway-X0842`; the hub retries
  `nodeN-uav-cvm:5762` every 2 s.
- Port in use on host `5760` → another stack is running; `down` it first.

## Limits

Vehicle emulation + aerpawlib + one QGC gateway + cross-node ZMQ coordination only.
No radio apps, channel effects, or shared platform checkpoints.
