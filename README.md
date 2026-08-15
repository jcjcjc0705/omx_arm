# omx_arm

ros2_control driver for DYNAMIXEL X-series robots, with an OpenMANIPULATOR-X
leader/follower teleoperation setup as the reference implementation.

ROS 2 Jazzy. Everything runs in Docker.

---

## Run it

Two images are published on every tag:

| Tag | Stage | Contains |
|---|---|---|
| `latest`, `v*` | runtime | prebuilt workspace only |
| `dev`, `dev-v*` | dev | plus colcon, rviz2, gdb, and a bind-mounted source tree |

### Runtime — just drive the arms

Nothing to build. Bring both arms up and start mirroring:

```bash
git clone https://github.com/jcjcjc0705/omx_arm.git
cd omx_arm
docker compose -f docker/docker-compose.yaml up follower leader teleop
```

Or one at a time:

```bash
docker compose -f docker/docker-compose.yaml up follower
```

### Dev — change the code

```bash
docker compose -f docker/docker-compose.yaml run --rm dev
```

Inside the container `/ws` is your working copy, so edits on the host apply
immediately:

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash

ros2 launch omx_bringup omx_f.launch.py          # follower
ros2 launch omx_bringup omx_l.launch.py          # leader
ros2 launch omx_teleop omx_teleop.launch.py      # mirror leader -> follower
```

Without hardware:

```bash
ros2 launch omx_bringup omx_f.launch.py use_mock_hardware:=true rviz:=true
```

---

## Hardware

Two OpenMANIPULATOR-X arms, each on an OpenRB-150 at 1 Mbaud. Motor IDs 11–16.
The arms mix XL430 and XL330 units; the driver detects which is which at
startup.

`docker/docker-compose.yaml` maps the boards by serial number so the two arms
never swap:

```yaml
devices:
  - /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_1E22...-if00:/dev/omx_follower
  - /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_01B5...-if00:/dev/omx_leader
```

For different boards, find yours with `ls -l /dev/serial/by-id/` and replace
those two lines. Inside the container the ports are always `/dev/omx_follower`
and `/dev/omx_leader`, which is what the launch files default to.

The follower holds torque and tracks the leader. The leader is back-drivable by
design — you drag it by hand — so its arm joints are declared read-only.

---

## Packages

| Package | Robot-specific | Contents |
|---|---|---|
| `dynamixel_msgs` | no | service definitions |
| `dynamixel_hardware` | no | serial bus, ros2_control plugin, offline CLI |
| `dynamixel_tools` | no | `dxl_cli`, `dxl_debug` |
| `omx_description` | yes | URDF, meshes, X-series xacro macro library |
| `omx_bringup` | yes | launch files, controller configuration |
| `omx_teleop` | yes | leader/follower mirror, dual-arm monitor |

Porting to another DYNAMIXEL robot means writing a new description and bringup
package. The three `dynamixel_*` packages are reused unchanged — they build in
a workspace containing nothing else.

---

## Tools

```bash
ros2 run dynamixel_tools dxl_cli dump 11              # full register table
ros2 run dynamixel_tools dxl_cli torque all off
ros2 run dynamixel_tools dxl_debug                    # curses TUI, jog joints
ros2 run omx_teleop dual_arm_monitor                  # both arms side by side
ros2 run omx_teleop dual_arm_monitor --joints gripper_joint_1
```

When the stack will not start — motor in Hardware Error, unknown ID, wrong baud
rate — use the offline tool, which needs no ROS runtime:

```bash
ros2 run dynamixel_hardware dxl_offline --port /dev/omx_follower scan
ros2 run dynamixel_hardware dxl_offline --port /dev/omx_follower reboot 13
```

Only one process may hold a serial port. Stop the stack before running
`dxl_offline`, or it will refuse to start.

---

## Teleoperation

`omx_teleop` mirrors leader joint positions onto the follower:

```
follower_cmd[i] = signs[i] * leader_pos[i] + offsets[i]
```

`signs` and `offsets` are launch parameters, not code — the two arms are
physically distinct and each joint may need a flip or a shift. The gripper runs
with `signs = -1`.

Mirroring stays disarmed until every joint is within `align_tolerance` of its
mapped target, so the follower cannot snap across a large pose gap at startup.
Watch it arm:

```
disarmed: gripper_joint_1 off by 0.684 rad
armed: mirroring live
```

---

## Releasing

Pushing any tag rebuilds both images and publishes them to GHCR via GitHub
Actions:

```bash
git tag v0.2.0 && git push origin v0.2.0
```

produces `:latest`, `:v0.2.0`, `:dev` and `:dev-v0.2.0`. Build either locally
with `--target`:

```bash
docker build -f docker/Dockerfile --target runtime -t omx_arm:runtime .
```

---

## Further reading

`INTERFACE.md` — URDF parameters, services, topics, unit conventions, and the
reasoning behind the design.
