# Copyright 2024 Universidad Politecnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Fleet configuration loader (Stage 18).

Resolves a declarative fleet YAML (vehicle count, ids, namespaces, ports, MAVLink
endpoints, backend, TF frames, per-UAV params) into concrete per-vehicle settings, so
a researcher changes the fleet by editing ``config/fleet.yaml`` — NOT source code.

Pure + ROS-free (only needs PyYAML), so it is unit-testable without a runtime.
"""

from __future__ import annotations

import os

import yaml

# Per-UAV parameter keys that a vehicle may override; defaults come from uav_defaults.
UAV_PARAM_KEYS = (
    "takeoff_altitude",
    "link_timeout",
    "position_update_min_distance",
    "position_update_min_yaw",
    "position_keepalive",
    "cmd_freq",
    "info_freq",
)

_DEFAULTS = {
    "namespace_prefix": "drone",
    "cmd_port_base": 15760,
    "tel_port_base": 15761,
    "port_stride": 2,
    "mavlink_port_base": 14550,
    "mavlink_port_stride": 10,
    "backend": "digital_twin",
    "frames": {"earth": "earth", "map": "map", "odom": "odom", "base": "base_link"},
    "uav_defaults": {},
    "vehicles": [],
}


def load_fleet(path=None, num_drones=None):
    """Load + resolve a fleet config.

    :param path: fleet YAML; None -> :data:`DEFAULT_FLEET_PATH`.
    :param num_drones: if set, override the vehicle count (grow with defaults /
        truncate) without editing the YAML (handy for quick launch tests).
    :returns: dict with ``frames``, ``backend`` and ``vehicles`` (list of resolved
        dicts: id, namespace, cmd_port, tel_port, conn, backend, params).
    """
    path = path or default_fleet_path()
    raw = {}
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}

    cfg = {k: raw.get(k, v) for k, v in _DEFAULTS.items()}
    frames = {**_DEFAULTS["frames"], **(raw.get("frames") or {})}
    uav_defaults = dict(cfg.get("uav_defaults") or {})

    vehicles = list(cfg["vehicles"] or [])
    if num_drones is not None:
        vehicles = (vehicles + [{} for _ in range(num_drones - len(vehicles))])[:int(num_drones)]

    resolved = []
    for i, v in enumerate(vehicles):
        v = v or {}
        prefix = cfg["namespace_prefix"]
        ns = v.get("namespace", f"{prefix}{i}")
        vid = v.get("id", ns)
        cmd_port = int(v.get("cmd_port", int(cfg["cmd_port_base"]) + i * int(cfg["port_stride"])))
        tel_port = int(v.get("tel_port", int(cfg["tel_port_base"]) + i * int(cfg["port_stride"])))
        conn = v.get("conn",
                     f"udpin://127.0.0.1:{int(cfg['mavlink_port_base']) + i * int(cfg['mavlink_port_stride'])}")
        backend = v.get("backend", cfg["backend"])
        params = {}
        for k in UAV_PARAM_KEYS:
            if k in v:
                params[k] = v[k]
            elif k in uav_defaults:
                params[k] = uav_defaults[k]
        resolved.append({
            "id": vid, "namespace": ns, "cmd_port": cmd_port, "tel_port": tel_port,
            "conn": conn, "backend": backend, "params": params,
        })

    fleet = {"frames": frames, "backend": cfg["backend"],
             "namespace_prefix": cfg["namespace_prefix"], "vehicles": resolved}
    validate_fleet(fleet)
    return fleet


def validate_fleet(fleet):
    """Fail loudly on ambiguous fleets (duplicate namespace / port / id)."""
    seen_ns, seen_port, seen_id = set(), set(), set()
    for v in fleet["vehicles"]:
        for key, val in (("namespace", v["namespace"]), ("id", v["id"])):
            bucket = seen_ns if key == "namespace" else seen_id
            if val in bucket:
                raise ValueError(f"duplicate vehicle {key}: {val!r}")
            bucket.add(val)
        for p in (v["cmd_port"], v["tel_port"]):
            if p in seen_port:
                raise ValueError(f"duplicate vehicle port: {p}")
            seen_port.add(p)
    return True


def default_fleet_path():
    """Path to the shipped default fleet.yaml (installed share or in-repo source)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("as2_platform_aerpaw"),
                         "config", "fleet.yaml")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.normpath(os.path.join(here, os.pardir, "config", "fleet.yaml"))
    return src if os.path.isfile(src) else None
