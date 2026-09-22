#!/usr/bin/env bash
#
# qgc_tunnel.sh - Bring an AERPAW vehicle's MAVLink (QGC) port to QGroundControl
#                 on your Mac so you can watch the drone fly live.
#
# WHERE TO RUN THIS:  ON YOUR MAC, while the AERPAW VPN is connected and
#                     QGroundControl is open. NOT on the C-VM/E-VM.
#
# WHY: QGC only needs a MAVLink endpoint it can reach. The drone sits behind the
#      AERPAW Message Filter, whose GCS/QGC port lives on the vehicle E-VM (or is
#      forwarded by the C-VM). This script forwards that port to your laptop.
#
# TWO WAYS TO SEE THE DRONE (pick one):
#   A) DIRECT (simplest, if your VPN routes to the E-VM IP):
#        In QGC: "UDP" autoconnect, or set the MAVLink endpoint to
#        <e-vm-ip>:<qgc-port>. No tunnel needed -> just skip this script.
#   B) SSH TUNNEL (this script; robust, works through NAT / when direct fails):
#        ssh -L a local port on the Mac to the E-VM's QGC port.
#        TCP is universally supported; UDP needs a recent OpenSSH (>= ~8.2,
#        macOS Ventura/Sonoma are fine). Start with PROTO=tcp.
#
# ----------------------------- FILL THESE IN ---------------------------------
# What to put in REMOTE_PORT: look at the vehicle profile's mavproxy/QGC line
#   (e.g. in startVehicle.sh a "--out tcpin://:<port>" / "udpin://:<port>") or
#   the OEO console "connect QGC" instructions. 5760 is only a placeholder.
SSH_USER="${SSH_USER:-root}"                      # AERPAW E-VM user (often root)
EVM_HOST="${EVM_HOST:-10.14.X.X}"                 # vehicle E-VM address (from console)
SSH_KEY="${SSH_KEY:-$HOME/.ssh/aerpaw_key}"       # your AERPAW SSH private key
QGC_PORT="${QGC_PORT:-5760}"                       # the E-VM's QGC/GCS port  <-- VERIFY
LOCAL_PORT="${LOCAL_PORT:-14550}"                  # QGC on the Mac connects here
PROTO="${PROTO:-tcp}"                              # tcp (safe) or udp
# ----------------------------------------------------------------------------

set -euo pipefail
log(){ printf '\033[1;36m==> %s\033[0m\n' "$*"; }
die(){ printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

[ "$EVM_HOST" != "10.14.X.X" ] || die "Set EVM_HOST to your vehicle E-VM address (from the OEO console)."

# Optional jump host: if you only reach E-VMs through the OEO console, set
#   JUMP="user@oeo-console"  and it is passed as -J.
JUMP_ARGS=()
[ -n "${JUMP:-}" ] && JUMP_ARGS=(-J "$JUMP")

if [ "$PROTO" = "udp" ]; then
  FORWARD="${LOCAL_PORT}/udp:127.0.0.1:${QGC_PORT}"   # OpenSSH >= ~8.2
else
  FORWARD="${LOCAL_PORT}:127.0.0.1:${QGC_PORT}"
fi

log "Forwarding Mac localhost:${LOCAL_PORT} (${PROTO}) -> ${EVM_HOST}:${QGC_PORT}"
log "Keep this running. Open QGC now."
if [ "$PROTO" = "udp" ]; then
cat <<EOF
QGC setup (PROTO=udp):  start QGC; it auto-listens on UDP. To be explicit set
LOCAL_PORT=14550 so QGC's default UDP autoconnect sees it. (UDP -L needs a recent
OpenSSH; macOS 13+ is fine.)
EOF
else
cat <<EOF
QGC setup (PROTO=tcp):  QGC -> Connect -> TCP -> host 127.0.0.1 port ${LOCAL_PORT}.
EOF
fi
echo "Ctrl-C to stop the tunnel."

# shellcheck disable=SC2029
exec ssh -N \
    -o ServerAliveInterval=15 -o ExitOnForwardFailure=yes \
    -i "$SSH_KEY" \
    "${JUMP_ARGS[@]}" \
    -L "$FORWARD" \
    "${SSH_USER}@${EVM_HOST}"