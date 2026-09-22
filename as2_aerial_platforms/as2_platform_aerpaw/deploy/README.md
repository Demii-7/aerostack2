# Deploy helpers (AERPAW Digital Twin)

Two scripts that live with the platform so they travel with the `git clone` on
the AERPAW side.

## `cvm_setup.sh` — run **on the AERPAW C-VM / vehicle E-VM**
Installs ROS 2 Humble, build deps, `aerpawlib`, clones this fork, builds the
workspace up to the experiment, and runs the platform unit tests. Idempotent.

```bash
# ssh into the AERPAW VM over your VPN first, then:
git clone -b main https://github.com/Demii-7/aerostack2.git ~/aerpaw_ws
bash ~/aerpaw_ws/as2_aerial_platforms/as2_platform_aerpaw/deploy/cvm_setup.sh
source ~/aerpaw_ws/install/setup.bash
```

Overrides: `WS_DIR`, `FORK_BRANCH`, `ROS_PKGSET` (use `ros-humble-ros-base` for a
slim install), `AERPAWLIB_DIR`, `ROS_DISTRO`.

After it finishes, launch Config C (real DT) on that VM:

```bash
source ~/aerpaw_ws/install/setup.bash
ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \
    use_aerpaw:=true \
    conn_drone0:=udpin://<e-vm-ip-0>:14550 \
    conn_drone1:=udpin://<e-vm-ip-1>:14550 \
    conn_drone2:=udpin://<e-vm-ip-2>:14550
```

The `conn_droneN` values are each vehicle's MAVLink **Message Filter** endpoint
(read from the OEO console / vehicle profile).

## `qgc_tunnel.sh` — run **on your Mac** (VPN up, QGC open)
Forwards a vehicle's QGC/MAVLink port to QGroundControl so you can watch it fly.

```bash
EVM_HOST=10.14.X.X SSH_USER=root SSH_KEY=~/.ssh/aerpaw_key QGC_PORT=<from-vehicle-profile> \
    bash as2_aerial_platforms/as2_platform_aerpaw/deploy/qgc_tunnel.sh
```

* Set `QGC_PORT` to the port the vehicle profile's mavproxy exposes for QGC
  (look for a `--out tcpin://:<port>` / `udpin://:<port>` in `startVehicle.sh`,
  or the console's "connect QGC" instructions). `5760` is only a placeholder.
* If your VPN routes directly to the E-VM, you can skip the tunnel and point QGC
  at `<e-vm-ip>:<qgc-port>`.
* Add `JUMP=user@oeo-console` if you can only reach E-VMs through the console.

See `docs/E2E_MANUAL.md` (Part 1 & Part 5) for the full AERPAW workflow.