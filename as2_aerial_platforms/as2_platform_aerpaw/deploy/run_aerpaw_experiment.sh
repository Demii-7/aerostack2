#!/usr/bin/env bash
#
# run_aerpaw_experiment.sh - AERPAW-first bring-up of the AERPAW+AeroStack2 stack.
#
# STAGE-2 OWNERSHIP: AERPAW is the TOP-LEVEL experiment platform.  This script is
# run *as the AERPAW experiment* on the AERPAW C-VM / vehicle E-VM (or against
# local SITL).  It is the thing that:
#
#     1. reserves / reaches the UAV + testbed resources (the MAVLink conns),
#     2. starts the AeroStack2 ROBOTICS SUBSYSTEM  (as2_stack.launch.py),
#     3. starts the per-drone aerpawlib bridge runners (AERPAW <-> AS2 adapter),
#     4. runs the researcher AS2 mission, and
#     5. tears the AS2 subsystem down when the experiment ends.
#
# AeroStack2 is a *subsystem* of the experiment: it is started here and never
# launches AERPAW itself.  (The inverted `ros2 launch ... aerpaw_digital_twin`
# path still exists but is a DEVELOPER SITL HARNESS ONLY - see that launch file.)
#
# USAGE:
#   ./run_aerpaw_experiment.sh                        # 3 drones, local SITL
#   NUM_DRONES=3 USE_AERPAW=true \
#     CONNS="udpin://10.14.1.11:14550 udpin://10.14.1.12:14550 udpin://10.14.1.13:14550" \
#     ./run_aerpaw_experiment.sh
#
# OVERRIDES (env): WS_DIR, NUM_DRONES, USE_AERPAW, RUN_MISSION, CONNS (space list,
# one MAVLink endpoint per drone), ROS_DISTRO.
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/aerpaw_ws}"
NUM_DRONES="${NUM_DRONES:-3}"
USE_AERPAW="${USE_AERPAW:-false}"      # true = real/Digital-Twin testbed, false = local SITL
RUN_MISSION="${RUN_MISSION:-true}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
# AERPAW environment (Stage 13): 'digital_twin' or 'physical'. They are
# capability-identical (same AERPAW environment / aerpawlib Drone API); switching the
# SAME experiment between them changes ONLY the CONNS endpoints below (DT: 10.14.x.x,
# physical: 192.168.32.x) + this safety label. No experiment/AS2/adapter code changes.
# Only meaningful when USE_AERPAW=true.
AERPAW_BACKEND="${AERPAW_BACKEND:-digital_twin}"
# Optional wireless/RF measurement poll for the closed loop (Stage 10), e.g.:
#   MEASURE_CHECKPOINT="sinr_db=radio/snr,rssi_dbm=radio/rssi"
# Empty => runner emits no measurements (nothing fabricated). Consumed by the
# experiment via <ns>/aerpaw/measurements.
MEASURE_CHECKPOINT="${MEASURE_CHECKPOINT:-}"
MEASURE_INTERVAL="${MEASURE_INTERVAL:-1.0}"
# Per-drone MAVLink endpoints (owned by AERPAW). Default = local SITL offsets.
if [ -z "${CONNS:-}" ]; then
  CONNS=""
  for i in $(seq 0 $((NUM_DRONES - 1))); do
    CONNS="$CONNS udpin://127.0.0.1:$((14550 + i * 10))"
  done
  CONNS="${CONNS# }"
fi
# Per-drone AERPAW vehicle names -> unique AS2 platform ids (one UAV = one platform).
# Default = drone{i}; override with VEHICLE_IDS="uav-north uav-east ...".
if [ -z "${VEHICLE_IDS:-}" ]; then
  VEHICLE_IDS=""
  for i in $(seq 0 $((NUM_DRONES - 1))); do
    VEHICLE_IDS="$VEHICLE_IDS drone${i}"
  done
  VEHICLE_IDS="${VEHICLE_IDS# }"
fi

# Per-drone IPC ports (default = fleet_config bases; else 15760/15761 + i*2).
if [ -z "${CMD_PORTS:-}" ]; then
  CMD_PORTS=""; TEL_PORTS=""
  for i in $(seq 0 $((NUM_DRONES - 1))); do
    CMD_PORTS="$CMD_PORTS $((15760 + i * 2))"
    TEL_PORTS="$TEL_PORTS $((15761 + i * 2))"
  done
  CMD_PORTS="${CMD_PORTS# }"; TEL_PORTS="${TEL_PORTS# }"
fi

# FLEET_CONFIG (Stage 18): when set, the whole fleet (count, ids, namespaces, ports,
# MAVLink endpoints, backend) is read from one YAML — no edits to this script.
FLEET_CONFIG="${FLEET_CONFIG:-}"
if [ -n "${FLEET_CONFIG}" ]; then
  log "Deriving fleet from ${FLEET_CONFIG}"
  # shellcheck disable=SC1090
  eval "$(FLEET_CONFIG_PATH="${FLEET_CONFIG}" python3 - <<'PY'
import os, shlex, sys
sys.path.insert(0, os.path.join(os.environ.get("WS_DIR", os.path.expanduser("~/aerpaw_ws")),
                                "as2_aerial_platforms/as2_platform_aerpaw/launch"))
import fleet_config as fc
f = fc.load_fleet(os.environ["FLEET_CONFIG_PATH"])
vids = [v["id"] for v in f["vehicles"]]
conns = [v["conn"] for v in f["vehicles"]]
cmd = [str(v["cmd_port"]) for v in f["vehicles"]]
tel = [str(v["tel_port"]) for v in f["vehicles"]]
print(f"NUM_DRONES={len(f['vehicles'])}")
print(f"VEHICLE_IDS={shlex.quote(' '.join(vids))}")
print(f"CONNS={shlex.quote(' '.join(conns))}")
print(f"CMD_PORTS={shlex.quote(' '.join(cmd))}")
print(f"TEL_PORTS={shlex.quote(' '.join(tel))}")
print(f"AERPAW_BACKEND={shlex.quote(f['vehicles'][0]['backend'])}")
print(f"USE_AERPAW={'true' if f['vehicles'][0]['backend'] != 'sitl' else 'false'}")
PY
)"
fi

log() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
PIDS=()
cleanup() {
  log "AERPAW experiment ending - tearing down AS2 subsystem + runners"
  for p in "${PIDS[@]:-}"; do
    kill -INT "$p" 2>/dev/null || true
  done
  for p in "${PIDS[@]:-}"; do
    ( wait "$p" 2>/dev/null ) || true
  done
}
trap cleanup EXIT INT TERM

# ---- 0. environment ---------------------------------------------------------
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}. Run deploy/cvm_setup.sh first." >&2
  exit 1
fi
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${WS_DIR}/install/setup.bash"
fi
export PATH="$HOME/.local/bin:$PATH"
command -v aerpawlib >/dev/null || { echo "aerpawlib not on PATH" >&2; exit 1; }

# ---- 1. AERPAW starts the AS2 ROBOTICS SUBSYSTEM (this is the key inversion) -
log "AERPAW -> starting AeroStack2 robotics subsystem (${NUM_DRONES} drones)"
FLEET_ARGS=()
[ -n "${FLEET_CONFIG}" ] && FLEET_ARGS+=(fleet_config:="${FLEET_CONFIG}")
ros2 launch as2_platform_aerpaw as2_stack.launch.py \
    num_drones:="${NUM_DRONES}" use_aerpaw:="${USE_AERPAW}" \
    platform_backend:="${AERPAW_BACKEND}" \
    vehicle_ids:="${VEHICLE_IDS}" \
    "${FLEET_ARGS[@]}" &
AS2_PID=$!
PIDS+=("$AS2_PID")

# AeroStack2 startup failure -> abort safely (don't launch vehicles into a dead
# robotics layer). Wait for the first platform node to appear, bounded by a timeout.
log "Waiting for the AeroStack2 subsystem to come up (platform node)..."
AS2_READY=0
for _ in $(seq 1 30); do
  if ! kill -0 "$AS2_PID" 2>/dev/null; then
    echo "ERROR: AeroStack2 subsystem exited during startup; aborting." >&2
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -q "/drone0/platform"; then
    AS2_READY=1; break
  fi
  sleep 1
done
if [ "$AS2_READY" != "1" ]; then
  echo "ERROR: AeroStack2 platform node did not come up within 30s; aborting." >&2
  exit 1
fi

# ---- 2. AERPAW starts the per-drone aerpawlib bridge runners ----------------
# shellcheck disable=SC2206
CONN_ARR=(${CONNS})
# shellcheck disable=SC2206
VID_ARR=(${VEHICLE_IDS})
# shellcheck disable=SC2206
CMD_ARR=(${CMD_PORTS})
# shellcheck disable=SC2206
TEL_ARR=(${TEL_PORTS})
RUNNER="${WS_DIR}/as2_aerial_platforms/as2_platform_aerpaw/aerpawlib_runner/aerpaw_as2_runner.py"
[ -f "$RUNNER" ] || RUNNER="$(ros2 pkg prefix as2_platform_aerpaw)/share/as2_platform_aerpaw/aerpawlib_runner/aerpaw_as2_runner.py"

for i in $(seq 0 $((NUM_DRONES - 1))); do
  conn="${CONN_ARR[$i]:-udpin://127.0.0.1:$((14550 + i * 10))}"
  vid="${VID_ARR[$i]:-drone${i}}"
  cmd_port="${CMD_ARR[$i]:-$((15760 + i * 2))}"
  tel_port="${TEL_ARR[$i]:-$((15761 + i * 2))}"
  log "AERPAW -> aerpawlib runner (drone${i} = ${vid}) conn=${conn} cmd=${cmd_port} tel=${tel_port}"
  AERPAW_ARGS=()
  [ "${USE_AERPAW}" = "true" ] || AERPAW_ARGS+=(--no-aerpaw-environment)
  MEAS_ARGS=()
  if [ -n "${MEASURE_CHECKPOINT}" ]; then
    MEAS_ARGS+=(--measure-checkpoint "${MEASURE_CHECKPOINT}" --measure-interval "${MEASURE_INTERVAL}")
  fi
  aerpawlib --api-version v2 \
      --script "$RUNNER" \
      --conn "$conn" \
      --vehicle drone \
      "${AERPAW_ARGS[@]}" \
      --cmd-port "$cmd_port" --tel-port "$tel_port" --vehicle-id "$vid" \
      "${MEAS_ARGS[@]}" &
  PIDS+=("$!")
done

# ---- 3. run the researcher AS2 mission (uses the AS2 subsystem) -------------
if [ "${RUN_MISSION}" = "true" ]; then
  log "Running AS2 mission (triangle_formation)"
  python3 "${WS_DIR}/as2_experiment_aerpaw_multiuav/missions/triangle_formation.py" &
  PIDS+=("$!")
fi

log "AERPAW experiment is running (AERPAW owns lifecycle; AS2 is a subsystem). Ctrl-C to stop."
wait
