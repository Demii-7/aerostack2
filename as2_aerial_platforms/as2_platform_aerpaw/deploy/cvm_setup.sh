#!/usr/bin/env bash
#
# cvm_setup.sh - One-shot setup of the AeroStack2 + AERPAW bridge on an
#                AERPAW C-VM or vehicle E-VM (the VM that can reach the
#                MAVLink Message Filter).
#
# WHERE TO RUN THIS:  ON THE AERPAW VM (C-VM / E-VM), NOT on your laptop and
#                     NOT on the hosted dev box.  You get there over the VPN:
#                       ssh -i <aerpaw_key> <user>@<cvm-or-evm-host>
#
# WHAT IT DOES:
#   1. Installs ROS 2 Humble (skipped if already present)
#   2. Installs system build deps (colcon, nlohmann-json, gtest, ...)
#   3. Clones/pulls this fork and aerpawlib
#   4. Builds the AS2 workspace up to the AERPAW experiment
#   5. Runs the platform unit tests
# It is idempotent: safe to re-run.
#
# OVERRIDES (env vars), e.g.:
#   WS_DIR=~/ws FORK_BRANCH=main ./cvm_setup.sh
#
set -euo pipefail

# ----------------------------- tunables ------------------------------------
FORK_URL="${FORK_URL:-https://github.com/Demii-7/aerostack2.git}"
FORK_BRANCH="${FORK_BRANCH:-main}"
WS_DIR="${WS_DIR:-$HOME/aerpaw_ws}"            # colcon workspace root (repo checkout)
AERPAWLIB_DIR="${AERPAWLIB_DIR:-$HOME/aerpawlib}"
AERPAWLIB_URL="${AERPAWLIB_URL:-https://github.com/AERPAW/aerpawlib.git}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_PKGSET="${ROS_PKGSET:-ros-humble-desktop}" # use ros-humble-ros-base for a slim build
# ---------------------------------------------------------------------------

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

# AERPAW E-VMs are often root; C-VMs may need sudo.
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo"; fi

# --------------------------- 0. sanity -------------------------------------
. /etc/os-release 2>/dev/null || true
if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "22.04" ]; then
  warn "Not Ubuntu 22.04 (got '${ID:-?} ${VERSION_ID:-?}'). ROS ${ROS_DISTRO} targets 22.04; continuing anyway."
fi

# --------------------------- 1. ROS 2 --------------------------------------
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  log "ROS 2 ${ROS_DISTRO} already installed - skipping"
else
  log "Installing ROS 2 ${ROS_DISTRO}"
  $SUDO apt-get update
  $SUDO apt-get install -y curl gnupg lsb-release software-properties-common
  # Add ROS apt repo + key
  $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
      | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  $SUDO apt-get update
  $SUDO apt-get install -y "$ROS_PKGSET" python3-colcon-common-extensions \
      python3-rosdep python3-pip
  # rosdep (idempotent)
  [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || $SUDO rosdep init || true
  rosdep update || warn "rosdep update failed (offline?); build may miss system deps"
fi

log "Sourcing /opt/ros/${ROS_DISTRO}/setup.bash for this script"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# --------------------------- 2. build deps ---------------------------------
log "Installing system build dependencies"
$SUDO apt-get install -y --no-install-recommends \
    git build-essential cmake ninja-build pkg-config \
    nlohmann-json3-dev libyaml-cpp-dev libeigen3-dev \
    python3-pip python3-setuptools python3-wheel \
    ros-"${ROS_DISTRO}"-ament-cmake-gtest

# aerpawlib must be importable by the SAME python that ROS uses.
PYBIN="$(command -v python3)"
log "Installing aerpawlib into ${PYBIN}"
if [ -d "$AERPAWLIB_DIR/.git" ]; then
  git -C "$AERPAWLIB_DIR" pull --ff-only || warn "aerpawlib pull failed; using existing"
else
  git clone "$AERPAWLIB_URL" "$AERPAWLIB_DIR"
fi
# Core only (SITL not needed: ArduPilot runs on the AERPAW vehicle side).
"$PYBIN" -m pip install --upgrade pip
"$PYBIN" -m pip install -e "$AERPAWLIB_DIR" || \
  "$PYBIN" -m pip install --break-system-packages -e "$AERPAWLIB_DIR"
# The runner is launched as the `aerpawlib` CLI; make sure it is on PATH.
export PATH="$HOME/.local/bin:$PATH"
command -v aerpawlib >/dev/null || warn "'aerpawlib' not on PATH; add ~/.local/bin to PATH before launching"

# --------------------------- 3. workspace ----------------------------------
log "Fetching workspace -> ${WS_DIR} (${FORK_BRANCH})"
if [ -d "$WS_DIR/.git" ]; then
  git -C "$WS_DIR" fetch origin
  git -C "$WS_DIR" checkout "$FORK_BRANCH"
  git -C "$WS_DIR" pull --ff-only origin "$FORK_BRANCH" || warn "pull failed; building current checkout"
else
  git clone -b "$FORK_BRANCH" "$FORK_URL" "$WS_DIR"
fi
cd "$WS_DIR"

# --------------------------- 4. build --------------------------------------
log "Building AS2 up to the AERPAW experiment (this pulls platform + est/ctrl/behaviors)"
colcon build --symlink-install \
    --packages-up-to as2_experiment_aerpaw_multiuav \
    --event-handlers console_direct+

# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

# --------------------------- 5. unit tests ---------------------------------
log "Running platform unit tests"
colcon test --packages-select as2_platform_aerpaw || true
colcon test-result --verbose

# --------------------------- 6. done ---------------------------------------
cat <<EOF

$(printf '\033[1;32m')SETUP COMPLETE$(printf '\033[0m')
Workspace : ${WS_DIR}
Overlay   : source ${WS_DIR}/install/setup.bash     (run in EVERY new shell)
Runner    : aerpawlib on PATH = $(command -v aerpawlib || echo 'MISSING - fix PATH')

NEXT (on this AERPAW VM):
  1) Find each vehicle's MAVLink endpoint from the OEO console / vehicle profile
     (the Message Filter script port), e.g. udpin://<e-vm-ip>:14550
  2) source ${WS_DIR}/install/setup.bash
  3) ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \\
         use_aerpaw:=true \\
         conn_drone0:=udpin://<e-vm-ip-0>:14550 \\
         conn_drone1:=udpin://<e-vm-ip-1>:14550 \\
         conn_drone2:=udpin://<e-vm-ip-2>:14550
  4) On your MAC: run qgc_tunnel.sh to bring the drone's QGC port to QGroundControl.
EOF