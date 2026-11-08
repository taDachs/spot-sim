# What the simulator does not do (yet)

The simulator serves the Spot API the spot_ros2 driver needs. This file lists
what it leaves out or only approximates, so it can be added when someone needs
it. Keep it current: when a slice adds a feature, remove it here; when you
find a new gap, add it.

**How gaps show up at runtime**

- **API methods the sim doesn't implement** answer `UNIMPLEMENTED`. The sim
  logs each one once (`[api] not simulated: ...`) and appends it to
  `/reports/unimplemented_api_calls.txt`. The `test_quiet_driver` e2e test
  fails if the driver called any of them.
- **Commands the sim doesn't simulate** answer `STATUS_UNSUPPORTED` with a
  message, for example `RobotCommand` with a stance request.
- **Services that are only stubs** give a valid but empty answer. They are
  marked *stub* below.

Code pointers: API semantics in `src/spot_sim_core` (`api_server.cpp`,
`commands.cpp`, `robot_state.cpp`, `images.cpp`), body and arm motion in the
`src/spot_sim_gz` plugins.

## Locomotion and body

| Missing | Current behaviour | Notes |
|---|---|---|
| `se2_trajectory_request` (driver: `trajectory` action, `navigate_to` goals in odom/vision) | UNSUPPORTED | Needs a go-to mode in `LocomotionCommand` plus a base controller; also needed for `WalkToObject*`. |
| Stance, follow-arm, battery-change pose, constrained manipulation | UNSUPPORTED | |
| Locomotion hints (`locomotion_mode`), stairs mode, swing height, terrain and obstacle params | Accepted, no effect | The kinematic base always trots the same way. |
| Body-control trajectories | Only the last point of `base_offset_rt_footprint` is used | No timing; the posture ramps at a fixed rate. |
| Collisions, terrain, stairs, slopes | The robot walks through objects; the ground is flat at z = 0 | `KinematicBaseSystem` turns physics off for the robot. Foot contact and `gpe` assume flat ground. |
| Falling, self-right from a fall | Never falls; self-right just sits | |
| Behavior / system / service faults | Always empty; `ClearBehaviorFault` returns NOT_CLEARED | |
| Vision odometry drift | `vision` == `odom` | |
| Battery drain, temperatures, metrics | Constant battery; `GetRobotMetrics` is a *stub* | |
| Low-level joint control (streaming services, ros2_control, `controllable:=true`) | Not offered | Out of scope by decision. |

## Arm (`ARM=true`)

| Missing | Current behaviour | Planned |
|---|---|---|
| Hand camera (`hand_color_image`, `hand_depth`, `hand_depth_in_hand_color_frame`) | Not served, so the driver's hand topics stay empty | Slice 10 |
| Grasping: `ManipulationApi` (PickObject*, WalkToObject*), `ManipulationApiFeedback` | UNIMPLEMENTED | Slice 11 (kinematic attach); WalkToObject needs the go-to base mode |
| Arm surface contact (driver: `arm_surface_contact` action) | UNIMPLEMENTED | Not planned |
| Arm impedance, drag, force / wrench trajectories (`force_trajectory`) | UNSUPPORTED | Not planned |
| Arm collisions (with the body, itself, the world) | None | |
| Hand velocity and force estimates in `ManipulatorState` | Not reported | |
| `STATUS_STALLED_HOLDING_ITEM`, carry-state rules when stowing with an item | Not modelled | |
| Gripper camera parameters | Stored and reported back, no effect on images | |
| Gripperless arm (`gripperless:=true`) | Not offered | Small: pass the xacro arg, drop `gripper-camera-param` |
| Arm in `InverseKinematicsService` | Always NO_SOLUTION_FOUND | Could reuse the slice 9 IK |
| Stow and carry poses | Approximations, not measured on a real robot (ready is the robot's) | |
| Arm trajectories: point velocities, `maximum_acceleration`, `pos_interpolation` / `ang_interpolation` | Ignored: linear in joints, linear + slerp in Cartesian space; too-fast points are slowed to the speed limits (1 rad/s, 0.5 m/s, 1 rad/s by default) | `spot_sim_core/arm_motion.cpp` |
| Root and task frames other than body, flat_body, odom, vision, gpe (e.g. `hand`, `arm0.link_sh0`, fiducials) | STATUS_UNKNOWN_FRAME (Cartesian velocity also accepts `hand`) | |
| Gaze: target and tool trajectories over time | Only their last point is used; without a tool trajectory the hand keeps its place relative to the body | |
| Unreachable poses | The arm stops at the last reachable setpoint and reports STALLED; no body assist, no planning around joint limits | |
| Arm velocity: angular velocity "relative to odom", acceleration limits | Angular velocity is applied in the hand frame relative to the body; no acceleration limit | |

## Cameras (`IMAGES=true`)

| Missing | Current behaviour |
|---|---|
| Fisheye optics | Pinhole cameras; images are already undistorted, as the driver assumes |
| `resize_ratio` other than 0 or 1 | Request rejected |
| Depth in formats other than RAW `DEPTH_U16` | Request rejected |
| Lighting, exposure, noise, motion blur | Plain Gazebo rendering |
| Spot CAM payload (PTZ, audio, LEDs, logpoints) | Not simulated; the driver only uses it with `initialize_spot_cam: true` (default false) |

## Services that are only stubs or missing

| Service | Current behaviour |
|---|---|
| GraphNav, MapProcessing (driver: `graph_nav_*`, `list_graph`, `navigate_to`) | GetLocalizationState and DownloadGraph return empty results, ClearGraph succeeds; everything else is UNIMPLEMENTED |
| Docking (driver: `dock`, `undock`) | Always undocked; docking fails with DOCK_NOT_FOUND |
| World objects | Only objects that clients add; no fiducials, docks or other perception |
| Choreography (driver: `execute_dance`, `upload_animation`, ...) | License reports no features; not simulated |
| SpotCheck, Mission (autowalk) | Listed in the directory so clients can be created; RPCs UNIMPLEMENTED |
| InverseKinematics (body) | Always NO_SOLUTION_FOUND |
| Payloads, data acquisition, network compute, logging | Not in the directory |

## Timing and infrastructure

- **Wall-clock timers:** the lease-staleness and E-Stop timeouts run on wall-clock time, because the SDK's keep-alive loops do. At RTF ≠ 1 they don't scale with simulation time.
- **Authentication:** one configured user, the test CA from spot_wrapper, and a fixed token.
- **One robot per sim:** a single robot per sim instance on port 443.
- **Gazebo world reset** (GUI reset button, `/world/.../control` reset): Gazebo re-creates `SpotApiSystem`, so the Spot API restarts from scratch. That means a new lease epoch, no E-Stop endpoint and the robot clock back at 0. The driver can't recover and keeps logging connection errors; restart the stack (`docker compose up -d --force-recreate`). To support resets, `SpotApiSystem` would implement `ISystemReset`: keep the server and firmware, reset the robot's pose and joints, and handle the clock jump (time sync and command end times).
