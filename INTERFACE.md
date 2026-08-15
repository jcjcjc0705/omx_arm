# Interface Reference

The contract between this repository and anything that drives it. Read
`README.md` first for how to run it.

---

## 1. Package layering

```
dynamixel_msgs        service definitions            no robot knowledge
dynamixel_hardware    serial bus + ros2_control plugin
dynamixel_tools       CLI / TUI / dual-arm monitor
──────────────────────────────────────────────────── reuse boundary
omx_description       URDF, meshes, ros2_control tags   OpenMANIPULATOR-X only
omx_bringup           launch files, controller YAML
omx_teleop            leader/follower mirroring
```

The three `dynamixel_*` packages contain no reference to OpenMANIPULATOR-X and
depend only on ROS 2 packages, `dynamixel_sdk`, and each other. They build in a
workspace that contains nothing else. A new robot needs new `*_description` and
`*_bringup` packages and nothing below the line.

---

## 2. URDF contract

### Hardware parameters

Declared once per `<ros2_control>` block.

| Parameter | Default | Meaning |
|---|---|---|
| `port_name` | `/dev/ttyUSB0` | serial device |
| `baud_rate` | `1000000` | X-series factory default is `57600` |

### Joint parameters

| Parameter | Required | Meaning |
|---|---|---|
| `device_id` | yes | DYNAMIXEL ID on the bus |
| `operating_mode` | no (`position`) | see below |
| `torque_enable` | no (`true`) | `false` leaves the joint back-drivable |

`operating_mode` accepts `position`, `extended_position`, `velocity`,
`current`, `current_based_position`, `pwm`.

### Register parameters

Any of these may be given as a `<param>` and is written to the motor during
`on_configure`. Names match the DYNAMIXEL control table.

```
drive_mode          homing_offset       temperature_limit
max_voltage_limit   min_voltage_limit   pwm_limit
current_limit       velocity_limit      max_position_limit
min_position_limit  shutdown            velocity_i_gain
velocity_p_gain     position_d_gain     position_i_gain
position_p_gain     goal_current        profile_acceleration
profile_velocity
```

`omx_description/urdf/dxl_x_series.xacro` wraps these into per-mode macros so a
joint is one line rather than twenty. Reuse that file for any X-series robot.

### Interfaces

Command: `position` (rad), when the joint is commandable.

State: `position`, `velocity`, `effort`, `temperature`, `voltage`, `pwm`,
`error`, `moving`, `homing_offset`.

---

## 3. Units

Positions are **radians centred on tick 2048**:

```
rad = (tick - 2048) * 2*pi / 4096
```

Tick 2048 is the X-series mechanical centre, so zero radians is the centred
pose and the sign matches the URDF joint axis. This differs from the common
`tick * 360 / 4096` convention, which is uncentred and in degrees: a gripper
reading 140 deg there reads -0.7 rad here.

Velocity is rad/s, converted from the motor's `0.229 rev/min` unit.

---

## 4. Services

Advertised by the hardware component under
`/<namespace>/<component_name_lowercased>/`. With
`<ros2_control name="OmxFSystem">` in the `omx_follower` namespace the prefix is
`/omx_follower/omxfsystem/`.

| Service | Type | Notes |
|---|---|---|
| `set_torque` | `dynamixel_msgs/SetTorque` | `id: 0` means every motor |
| `reboot` | `dynamixel_msgs/Reboot` | clears Hardware Error Status |
| `set_zero` | `dynamixel_msgs/SetZero` | writes present position as homing offset |
| `read_register` | `dynamixel_msgs/ReadRegister` | arbitrary address/size |
| `write_register` | `dynamixel_msgs/WriteRegister` | arbitrary address/size |

All replies carry `bool success` and `string message`.

---

## 5. Topics

| Topic | Direction | Type |
|---|---|---|
| `<ns>/joint_states` | out | `sensor_msgs/JointState` |
| `<ns>/dynamic_joint_states` | out | `control_msgs/DynamicJointState` |
| `<ns>/position_controller/commands` | in | `std_msgs/Float64MultiArray` |

`joint_states` carries only position/velocity/effort and is always ordered
alphabetically. Everything else — temperature, voltage, error — is on
`dynamic_joint_states`. The `joints` parameter on `joint_state_broadcaster`
filters which joints appear; it does not control ordering.

Command array order follows the controller's `joints` parameter, which is not
alphabetical. Match by name, never by index.

---

## 6. Tools

| Tool | Needs a running stack | Use |
|---|---|---|
| `dxl_cli` | yes | one-shot register read/write, torque, reboot, zero |
| `dxl_debug` | yes | curses TUI, jog joints, watch state |
| `dual_arm_monitor` | yes | leader and follower side by side, read-only |
| `dxl_offline` | **no** | rescue: scan, dump, reboot when the stack is down |

`dxl_offline` is a C++ executable linking only the bus library. Use it when a
motor is stuck in Hardware Error, has an unknown ID, or is at the wrong baud
rate — cases the ROS services cannot reach because the stack will not start.
It takes the same port lock, so stop the stack first.

The three ROS tools discover the controller and hardware component from the
graph. With two stacks running they list every candidate and pick the
alphabetically first; pass `--controller` / `--component` to choose.

---

## 7. Known limitations

- **Joint limits are ±2π**, inherited unchanged from the ROBOTIS description.
  They are wide enough to offer no real protection; the firmware
  `min/max_position_limit` registers and the mechanism itself are what stop the
  arm. Set both to measured values before deploying.
- **`effort` reads 0.0.** See section 6.
- **No real-time scheduling.** The container runs at default priority, which is
  adequate at the 50 Hz used here. Pushing to 100 Hz or beyond wants
  `cap_add: SYS_NICE` and `ulimits: rtprio: 99`.
