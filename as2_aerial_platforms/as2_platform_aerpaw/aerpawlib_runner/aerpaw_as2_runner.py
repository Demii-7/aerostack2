#!/usr/bin/env python3
"""Aerpawlib runner bridging AS2 to aerpawlib via UDP/JSON IPC.

This script is a plain aerpawlib experimenter script: it defines exactly one
BasicRunner subclass and is executed by the aerpawlib CLI, e.g.

  aerpawlib --api-version v2 \\
      --script aerpaw_as2_runner.py \\
      --conn udpin://127.0.0.1:14550 \\
      --vehicle drone --no-aerpaw-environment \\
      --cmd-port 15760 --tel-port 15761

The custom ``--cmd-port`` / ``--tel-port`` options are picked up by
``initialize_args()`` via aerpawlib's extra-arguments forwarding mechanism
(unknown CLI args are passed to the runner).

Commands received from the C++ platform node over UDP (one JSON object per
datagram):

  {"cmd": "arm"} {"cmd": "disarm"}
  {"cmd": "takeoff", "alt": 25.0} {"cmd": "land"}
  {"cmd": "set_velocity", "vx":0, "vy":0, "vz":0}
  {"cmd": "goto_ned", "north":0, "east":0, "down":-25, "heading":0,
   "home_lat": ..., "home_lon": ...}
  {"cmd": "kill"} {"cmd": "stop"}

Telemetry is streamed back to the C++ node at 20 Hz.

This runner is AERPAW-side and intentionally carries NO ROS 2 knowledge: its only
coupling to the robotics layer is the ROS-free UDP/JSON interface described above
(Stage 14). If the AERPAW experiment wants this process to also bring up the
robotics subsystem, pass ``--as2-subsystem`` with an OPAQUE ``--as2-subsystem-cmd``
that the runner merely starts as a child and tears down on exit; the runner does not
interpret it. Normally the AERPAW orchestrator (deploy/run_aerpaw_experiment.sh) does
the bring-up, so the runner stays a pure bridge.
"""

import argparse
import asyncio
import contextlib
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time

from aerpawlib.v2 import BasicRunner, Coordinate, Drone, VectorNED, entrypoint

CMD_PORT = 15760
TEL_PORT = 15761
LOCALHOST = "127.0.0.1"

# AERPAW <-> Adapter wire-protocol version (IF-1). Keep in lockstep with
# aerpaw_platform::kProtocolVersion (ipc_bridge.hpp). MAJOR=incompatible, MINOR=additive.
PROTOCOL_VERSION = 1


# --- AERPAW-side coordinate/unit helpers -------------------------------------
# The canonical, bidirectional frame mapping (ENU<->NED, ENU yaw <-> NED heading,
# NED euler -> ENU quaternion, etc.) lives in the C++ adapter:
#   as2_platform_aerpaw/include/as2_platform_aerpaw/frame_conversions.hpp
# This runner sits on the AERPAW side of the seam and only does the two conversions
# needed to speak MAVSDK/aerpawlib's native NED/radians convention: heading rad->deg
# and relative-alt -> AMSL, plus building an absolute target from a home+NED offset.
def heading_deg_from_yaw_ned_rad(yaw_ned_rad: float) -> float:
    """aerpawlib attitude.yaw (NED, radians, 0=North CW) -> heading degrees."""
    return math.degrees(yaw_ned_rad)


def alt_msl_from_relative(rel_alt_m: float, home_amsl_m: float) -> float:
    """aerpawlib position.alt is home-relative metres -> altitude above mean sea level."""
    return rel_alt_m + home_amsl_m


def goto_target_from_ned(home_lat: float, home_lon: float, north: float, east: float, down: float):
    """Absolute WGS-84 target = fixed home (ENU origin) + NED offset (aerpawlib arithmetic)."""
    return Coordinate(home_lat, home_lon, 0.0) + VectorNED(north, east, down)

# Optional subsystem bring-up command, supplied by the AERPAW experiment when it wants
# this runner to also start the robotics subsystem (see --as2-subsystem). It is
# deliberately OPAQUE: the runner starts/stops it as a child and never interprets it,
# so this AERPAW-side file carries NO ROS 2 knowledge (Stage 14). Empty = disabled.
DEFAULT_AS2_SUBSYSTEM_CMD = ""


class AerpawAs2Runner(BasicRunner):
    """Bridge aerpawlib commands to the AS2 AERPAW platform IPC socket."""

    def __init__(self):
        super().__init__()
        self._cmd_port = CMD_PORT
        self._tel_port = TEL_PORT
        self._vehicle_id = ""
        self._cmd_sock = None
        self._tel_sock = None
        self._tel_target = None
        # AERPAW-owned AS2 robotics subsystem (opt-in, see --as2-subsystem).
        self._as2_subsystem = False
        self._as2_subsystem_cmd = DEFAULT_AS2_SUBSYSTEM_CMD
        self._as2_proc = None
        # Wireless/RF measurements (opt-in): map "wireless metric name" -> "AERPAW
        # checkpoint string key" to poll. Empty => disabled. Never fabricated: if a
        # value cannot be read, the metric is simply omitted from the payload.
        self._measure_checkpoint: dict[str, str] = {}
        self._measure_interval = 1.0
        # Online reference updates (Stage 11): movement commands run in a background
        # worker so the command loop never blocks on arrival and a new in-flight
        # reference supersedes the previous one (latest-wins). Not static-plan-only.
        self._move_evt = None
        self._pending_move = None
        self._cur_move = None

    def initialize_args(self, args: list[str]) -> None:
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument("--cmd-port", type=int, default=self._cmd_port)
        parser.add_argument("--tel-port", type=int, default=self._tel_port)
        parser.add_argument("--vehicle-id", default="",
                            help="AERPAW vehicle id echoed in telemetry/status")
        parser.add_argument("--as2-subsystem", action="store_true",
                            help="AERPAW launches the AS2 robotics subsystem as a child")
        parser.add_argument("--as2-subsystem-cmd", default=self._as2_subsystem_cmd,
                            help="Shell command that stands up the AS2 subsystem")
        parser.add_argument("--measure-checkpoint", default="",
                            help="Wireless metric->AERPAW checkpoint-string key, e.g. "
                                 "'sinr_db=radio/snr,rssi_dbm=radio/rssi' (poll & forward)")
        parser.add_argument("--measure-interval", type=float, default=1.0,
                            help="Seconds between checkpoint measurement polls")
        parsed = parser.parse_args(args)
        self._cmd_port = parsed.cmd_port
        self._tel_port = parsed.tel_port
        self._vehicle_id = parsed.vehicle_id
        self._as2_subsystem = parsed.as2_subsystem
        self._as2_subsystem_cmd = parsed.as2_subsystem_cmd
        self._measure_interval = max(0.1, parsed.measure_interval)
        self._measure_checkpoint = {}
        for pair in parsed.measure_checkpoint.split(","):
            pair = pair.strip()
            if "=" in pair:
                name, key = pair.split("=", 1)
                self._measure_checkpoint[name.strip()] = key.strip()

        # UDP is connectionless: bind only on the receiving side.
        # Command channel: C++ sends here.  Telemetry channel: we send to C++.
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.bind((LOCALHOST, self._cmd_port))
        self._cmd_sock.setblocking(False)

        self._tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tel_target = (LOCALHOST, self._tel_port)

        print(f"[aerpaw_as2_runner] IPC ready: cmd_port={self._cmd_port} tel_target={self._tel_target}")

    async def _recv_cmd(self) -> dict | None:
        loop = asyncio.get_event_loop()
        try:
            data, _addr = await loop.run_in_executor(None, self._cmd_sock.recvfrom, 65536)
            return json.loads(data.decode())
        except (BlockingIOError, json.JSONDecodeError):
            return None

    def _send_json(self, obj: dict) -> None:
        """Send one typed JSON datagram to the C++ platform (telemetry/status/error).

        Stamps the wire protocol version on every outbound message (IF-1).
        """
        obj.setdefault("proto", PROTOCOL_VERSION)
        try:
            self._tel_sock.sendto(json.dumps(obj).encode(), self._tel_target)
        except Exception:
            pass

    def _send_status(self, message: str) -> None:
        self._send_json({"type": "status", "vehicle_id": self._vehicle_id,
                         "message": message, "ts": time.time()})

    def _send_error(self, command: str, message: str) -> None:
        self._send_json({"type": "error", "vehicle_id": self._vehicle_id,
                         "command": command, "message": message, "ts": time.time()})

    async def _stream_telemetry(self, drone: Drone) -> None:
        while True:
            try:
                pos = drone.position
                vel = drone.velocity
                att = drone.attitude
                tel = {
                    "type": "telemetry",
                    "vehicle_id": self._vehicle_id,
                    "mavlink_sysid": getattr(drone, "system_id", 0) or 0,
                    "lat": pos.lat,
                    "lon": pos.lon,
                    "alt_msl": alt_msl_from_relative(pos.alt, drone.home_amsl),
                    "rel_alt": pos.alt,
                    "vx_ned": vel.north,
                    "vy_ned": vel.east,
                    "vz_ned": vel.down,
                    # Full attitude (MAVSDK NED/FRD euler, radians) - authoritative.
                    "roll_rad": att.roll,
                    "pitch_rad": att.pitch,
                    "yaw_ned_rad": att.yaw,
                    # Legacy keys (roll/pitch rad, yaw deg-from-North) kept for compat.
                    "roll": att.roll,
                    "pitch": att.pitch,
                    "yaw": heading_deg_from_yaw_ned_rad(att.yaw),
                    "armed": drone.armed,
                    "mode": drone.mode,
                    "connected": drone.connected,
                    "ts": time.time(),
                }
                # Optional state, only added when aerpawlib actually exposes it
                # (never invented). Callers treat absence as "not available".
                gps = getattr(drone, "gps", None)
                if gps is not None:
                    tel["gps_fix_type"] = int(getattr(gps, "fix_type", 0) or 0)
                    tel["gps_satellites"] = int(getattr(gps, "satellites_visible", 0) or 0)
                bat = getattr(drone, "battery", None)
                if bat is not None:
                    tel["battery_voltage"] = float(getattr(bat, "voltage", 0.0) or 0.0)
                    tel["battery_current"] = float(getattr(bat, "current", 0.0) or 0.0)
                    tel["battery_level"] = float(getattr(bat, "level", 0.0) or 0.0)
                for flag in ("armable", "ekf_ready"):
                    val = getattr(drone, flag, None)
                    if val is not None:
                        tel[flag] = bool(val)
                self._send_json(tel)
            except Exception:
                pass
            await asyncio.sleep(0.05)

    def _read_checkpoint(self, key: str):
        """Read one AERPAW checkpoint-string value. Returns None if unavailable.

        Uses aerpawlib's real checkpoint API; never invents a value. Any error
        (not connected, missing key, non-numeric) yields None so the metric is
        simply omitted from the payload.
        """
        try:
            from aerpawlib.v2.aerpaw import AerpawPlatform
            raw = AerpawPlatform().checkpoint_check_string(key)
        except Exception:
            return None
        if raw is None:
            return None
        try:
            return float(raw)
        except (TypeError, ValueError):
            return raw  # pass through non-numeric (e.g. channel id) as a string

    async def _stream_measurements(self) -> None:
        """Poll configured AERPAW metrics and forward them (transport only)."""
        while True:
            metrics = {}
            for name, key in self._measure_checkpoint.items():
                val = await asyncio.get_event_loop().run_in_executor(None, self._read_checkpoint, key)
                if val is not None:
                    metrics[name] = val
            if metrics:
                self._send_json({
                    "type": "measurement",
                    "vehicle_id": self._vehicle_id,
                    "metrics": metrics,
                    "ts": time.time(),
                })
            await asyncio.sleep(self._measure_interval)

    def _start_as2_subsystem(self) -> None:
        """Stand up the AeroStack2 robotics subsystem as an AERPAW child process.

        AERPAW owns the experiment lifecycle, so the AS2 subsystem's lifetime is
        tied to this runner: when AERPAW starts the experiment the subsystem comes
        up, and it is torn down in ``_stop_as2_subsystem`` when the experiment
        ends.  This is the AERPAW->AS2 direction; AS2 never launches AERPAW.
        """
        if not self._as2_subsystem or self._as2_proc is not None:
            return
        if not self._as2_subsystem_cmd:
            print("[aerpaw_as2_runner] --as2-subsystem set but no --as2-subsystem-cmd; "
                  "skipping (the AERPAW orchestrator normally starts the subsystem)")
            return
        print(f"[aerpaw_as2_runner] AERPAW starting subsystem (opaque cmd): {self._as2_subsystem_cmd}")
        self._as2_proc = subprocess.Popen(
            self._as2_subsystem_cmd, shell=True, start_new_session=True,
            env=os.environ.copy(),
        )

    def _stop_as2_subsystem(self) -> None:
        """Terminate the AERPAW-owned AS2 subsystem (SIGINT then SIGKILL)."""
        if self._as2_proc is None:
            return
        print("[aerpaw_as2_runner] AERPAW stopping AS2 subsystem")
        try:
            os.killpg(os.getpgid(self._as2_proc.pid), signal.SIGINT)
            self._as2_proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self._as2_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        except ProcessLookupError:
            pass
        self._as2_proc = None

    # Movement ops run in a background worker (latest-wins) so the command loop never
    # blocks on arrival and a researcher can update the reference during flight.
    _MOVE_OPS = ("takeoff", "land", "set_velocity", "goto_ned")

    async def _exec_move(self, drone: Drone, cmd: dict) -> None:
        """Await one movement command against aerpawlib (runs in the worker task)."""
        op = cmd.get("cmd")
        if op == "takeoff":
            await drone.takeoff(altitude=cmd.get("alt", 25.0))
        elif op == "land":
            await drone.land()
        elif op == "set_velocity":
            # aerpawlib marks set_velocity [NOT SUPPORTED] on the AERPAW testbed
            # (E-VM filter blocks offboard velocity); the AS2 adapter gates it out for
            # DT/physical, so it only reaches here in local SITL.
            await drone.set_velocity(
                VectorNED(float(cmd.get("vx", 0.0)), float(cmd.get("vy", 0.0)),
                          float(cmd.get("vz", 0.0))))
        elif op == "goto_ned":
            # AS2 sends absolute NED offsets about a fixed odom origin -> absolute
            # WGS-84 coordinate. Non-blocking so a newer reference can supersede it.
            target = goto_target_from_ned(
                float(cmd["home_lat"]), float(cmd["home_lon"]),
                float(cmd["north"]), float(cmd["east"]), float(cmd["down"]))
            await drone.goto_coordinates(target, target_heading=cmd.get("heading"))

    def _request_move(self, cmd: dict) -> None:
        """Enqueue the newest movement reference (coalesce: latest wins)."""
        self._pending_move = cmd
        if self._move_evt is not None:
            self._move_evt.set()

    async def _movement_worker(self, drone: Drone) -> None:
        """Single-flight executor: always run the newest pending reference; supersede."""
        while True:
            await self._move_evt.wait()
            self._move_evt.clear()
            cmd = self._pending_move
            self._pending_move = None
            if cmd is None:
                continue
            # Preempt: stop waiting for any in-flight leg so the new target wins ASAP.
            if self._cur_move is not None and not self._cur_move.done():
                self._cur_move.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await self._cur_move
            self._cur_move = asyncio.ensure_future(self._exec_move(drone, cmd))
            try:
                await self._cur_move
                self._send_status(f"ok {cmd.get('cmd')}")
            except asyncio.CancelledError:
                continue  # superseded by a newer reference
            except Exception as e:
                print(f"[aerpaw_as2_runner] Move {cmd!r} failed: {e}", file=sys.stderr)
                self._send_error(str(cmd.get("cmd")), str(e))

    async def _dispatch_command(self, drone: Drone, cmd: dict) -> None:
        """Handle a command from the recv loop WITHOUT blocking on arrival."""
        op = cmd.get("cmd")
        if op in self._MOVE_OPS:
            self._request_move(cmd)         # non-blocking; worker executes newest
        elif op == "arm":
            await drone.set_armed(True)
        elif op == "disarm":
            await drone.set_armed(False)
        elif op == "kill":
            # aerpawlib/ArduPilot has no irreversible motor-stop; disarm is strongest.
            if self._cur_move is not None and not self._cur_move.done():
                self._cur_move.cancel()
            await drone.set_armed(False)
            with contextlib.suppress(Exception):
                await drone.stop_velocity()
        elif op == "stop":
            if self._cur_move is not None and not self._cur_move.done():
                self._cur_move.cancel()
            try:
                await drone.stop_velocity()
            except Exception:
                await drone.set_velocity(VectorNED(0.0, 0.0, 0.0))
        else:
            self._send_error(str(op), f"unknown command: {cmd!r}")
            print(f"[aerpaw_as2_runner] Unknown command: {cmd}", file=sys.stderr)
            return
        self._send_status(f"ok {op}")

    @entrypoint
    async def run(self, drone: Drone):
        """Receive commands and stream telemetry until interrupted."""
        self._start_as2_subsystem()
        self._move_evt = asyncio.Event()
        tel_task = asyncio.create_task(self._stream_telemetry(drone))
        move_task = asyncio.create_task(self._movement_worker(drone))
        meas_task = (asyncio.create_task(self._stream_measurements())
                     if self._measure_checkpoint else None)
        try:
            while True:
                cmd = await self._recv_cmd()
                if cmd is None:
                    await asyncio.sleep(0.01)
                    continue
                try:
                    await self._dispatch_command(drone, cmd)
                except Exception as e:
                    print(f"[aerpaw_as2_runner] Command {cmd!r} failed: {e}", file=sys.stderr)
                    self._send_error(str(cmd.get("cmd")), str(e))
        finally:
            tel_task.cancel()
            move_task.cancel()
            if meas_task is not None:
                meas_task.cancel()
            self._stop_as2_subsystem()


if __name__ == "__main__":
    sys.exit("Run this script through the aerpawlib CLI, not directly.")
