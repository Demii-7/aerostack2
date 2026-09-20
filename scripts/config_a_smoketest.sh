#!/usr/bin/env bash
# Config A smoke test for as2_experiment_aerpaw_multiuav (AS2 Multirotor Simulator).
#
# Launches N drones + full AS2 stack + the triangle_formation mission, waits for
# the mission to finish, tears everything down, and prints PASS/FAIL.
#
# Usage:
#   scripts/config_a_smoketest.sh [NUM_DRONES] [TIMEOUT_SECONDS]
# Examples:
#   scripts/config_a_smoketest.sh          # 3 drones, 120s cap
#   scripts/config_a_smoketest.sh 1 60      # 1 drone, 60s cap
#
# Exit code: 0 = PASS, 1 = FAIL. Full log path is printed at the end.
set -o pipefail

WS="${AS2_WS:-/home/jovyan/work/project}"
ENV_NAME="humble"
NUM_DRONES="${1:-3}"
TIMEOUT="${2:-120}"
LOG="$(mktemp /tmp/configA.XXXXXX.log)"

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# --- preflight ---
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[smoke] ERROR: $WS/install/setup.bash not found."
  echo "        Build first:"
  echo "          conda run -n humble bash -lc 'cd $WS && colcon build \\"
  echo "            --packages-up-to as2_experiment_aerpaw_multiuav \\"
  echo "            --packages-ignore as2_platform_gazebo as2_gazebo_assets \\"
  echo "            --executor sequential --cmake-args -DBUILD_TESTING=OFF'"
  exit 2
fi

# --- environment ---
# shellcheck disable=SC1091
source /opt/conda/etc/profile.d/conda.sh
conda activate "$ENV_NAME"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
cd "$WS" || exit 2

echo "[smoke] ws=$WS env=$ENV_NAME drones=$NUM_DRONES timeout=${TIMEOUT}s log=$LOG"

# --- launch (new session so we can SIGINT the whole group) ---
setsid ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py \
  num_drones:="$NUM_DRONES" >"$LOG" 2>&1 &
LPID=$!
echo "[smoke] launched pid=$LPID"

# real failure markers (NOT the benign post-'Done!' auto-spin ExternalShutdown)
FAILRE='process has died|terminate called|ModuleNotFoundError|No module named|Invalid type.*use_sim_time|Statically typed parameter|must be initialized'

PASS=0
for ((t=0; t<TIMEOUT; t+=2)); do
  if grep -q '\[triangle\] Done!' "$LOG"; then PASS=1; break; fi
  if grep -qE "$FAILRE" "$LOG"; then echo "[smoke] failure marker detected"; break; fi
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[smoke] launcher exited before Done!"; break; fi
  sleep 2
done

# --- teardown: SIGINT the launch process group, then reap strays ---
kill -INT -- -"$LPID" 2>/dev/null || kill -INT "$LPID" 2>/dev/null
sleep 4
pkill -9 -f 'work/project/[i]nstall/as2' 2>/dev/null
pkill -9 -f 'sim_[m]ultirotor.launch.py' 2>/dev/null
pkill -9 -f 'triangle_[f]ormation.py' 2>/dev/null

# --- report ---
started=$(grep -c 'process started' "$LOG")
died=$(grep -c 'process has died' "$LOG")
plugins=$(grep -c 'PLUGIN LOADED' "$LOG")
echo "================= Config A smoke test ================="
echo "nodes started          : $started"
echo "processes that died    : $died"
echo "controller plugin loads: $plugins"
echo "---- mission log ----"
grep -E '\[triangle\]' "$LOG" || true
if [ "$PASS" -eq 1 ] && [ "$died" -eq 0 ]; then
  echo "RESULT: PASS"; rc=0
else
  echo "RESULT: FAIL"; rc=1
  echo "---- last 30 log lines ----"; tail -n 30 "$LOG"
fi
echo "full log: $LOG"
exit $rc
