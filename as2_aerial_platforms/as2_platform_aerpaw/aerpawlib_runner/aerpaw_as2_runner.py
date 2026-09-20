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
"""

import argparse
import asyncio
import json
import math
import socket
import sys
import time

from aerpawlib.v2 import BasicRunner, Coordinate, Drone, VectorNED, entrypoint

CMD_PORT = 15760
TEL_PORT = 15761
LOCALHOST = "127.0.0.1"


class AerpawAs2Runner(BasicRunner):
    """Bridge aerpawlib commands to the AS2 AERPAW platform IPC socket."""

    def __init__(self):
        super().__init__()
        self._cmd_port = CMD_PORT
        self._tel_port = TEL_PORT
        self._cmd_sock = None
        self._tel_sock = None
        self._tel_target = None

    def initialize_args(self, args: list[str]) -> None:
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument("--cmd-port", type=int, default=self._cmd_port)
        parser.add_argument("--tel-port", type=int, default=self._tel_port)
        parsed = parser.parse_args(args)
        self._cmd_port = parsed.cmd_port
        self._tel_port = parsed.tel_port

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

    async def _stream_telemetry(self, drone: Drone) -> None:
        while True:
            try:
                pos = drone.position
                vel = drone.velocity
                att = drone.attitude
                tel = {
                    "lat": pos.lat,
                    "lon": pos.lon,
                    "alt_msl": pos.alt + drone.home_amsl,
                    "rel_alt": pos.alt,
                    "vx_ned": vel.north,
                    "vy_ned": vel.east,
                    "vz_ned": vel.down,
                    "roll": att.roll,
                    "pitch": att.pitch,
                    "yaw": math.degrees(att.yaw),  # aerpawlib stores rad -> send deg from North
                    "armed": drone.armed,
                    "mode": drone.mode,
                    "ts": time.time(),
                }
                self._tel_sock.sendto(json.dumps(tel).encode(), self._tel_target)
            except Exception:
                pass
            await asyncio.sleep(0.05)

    async def _handle_command(self, drone: Drone, cmd: dict) -> None:
        op = cmd.get("cmd")
        if op == "arm":
            await drone.set_armed(True)
        elif op == "disarm":
            await drone.set_armed(False)
        elif op == "takeoff":
            await drone.takeoff(altitude=cmd.get("alt", 25.0))
        elif op == "land":
            await drone.land()
        elif op == "set_velocity":
            vx = float(cmd.get("vx", 0.0))
            vy = float(cmd.get("vy", 0.0))
            vz = float(cmd.get("vz", 0.0))
            await drone.set_velocity(VectorNED(vx, vy, vz))
        elif op == "goto_ned":
            # AS2 sends absolute NED offsets relative to a fixed odom origin
            # (home_lat/home_lon captured at the C++ platform's first telemetry).
            # Convert to absolute WGS-84 coordinate for goto_coordinates.
            north = float(cmd["north"])
            east = float(cmd["east"])
            down = float(cmd["down"])
            heading = cmd.get("heading")
            home_lat = float(cmd["home_lat"])
            home_lon = float(cmd["home_lon"])
            home = Coordinate(home_lat, home_lon, 0.0)
            target = home + VectorNED(north, east, down)
            await drone.goto_coordinates(target, target_heading=heading)
        elif op == "kill":
            # NOTE: aerpawlib/ArduPilot has no true irreversible emergency motor
            # stop (AS2 kill-switch semantics).  This is the strongest available
            # action: disarm + zero velocity.  Documented limitation.
            await drone.set_armed(False)
            try:
                await drone.stop_velocity()
            except Exception:
                pass
        elif op == "stop":
            try:
                await drone.stop_velocity()
            except Exception:
                await drone.set_velocity(VectorNED(0.0, 0.0, 0.0))
        else:
            print(f"[aerpaw_as2_runner] Unknown command: {cmd}", file=sys.stderr)

    @entrypoint
    async def run(self, drone: Drone):
        """Receive commands and stream telemetry until interrupted."""
        tel_task = asyncio.create_task(self._stream_telemetry(drone))
        try:
            while True:
                cmd = await self._recv_cmd()
                if cmd is None:
                    await asyncio.sleep(0.01)
                    continue
                try:
                    await self._handle_command(drone, cmd)
                except Exception as e:
                    print(f"[aerpaw_as2_runner] Command {cmd!r} failed: {e}", file=sys.stderr)
        finally:
            tel_task.cancel()


if __name__ == "__main__":
    sys.exit("Run this script through the aerpawlib CLI, not directly.")
