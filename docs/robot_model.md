# How the simulated robot moves

The Spot API side of the simulator (`spot_sim_core`, the robot's "firmware")
decides *what* the robot should do. Gazebo plugins decide *how* the body,
legs and arm get there. They talk through plain structs stored as Gazebo
components on the robot model (`src/spot_sim_gz/include/spot_sim_gz/components.hpp`).
Every plugin is swappable as long as it keeps that contract.

```
                    power, sit / stand / stop, posture
SpotApiSystem ─────────────────────────────────────────────▶ base plugin
 (firmware)            SpotLocomotionCommand                 (KinematicBaseSystem today,
     │                                                         an RL policy later)
     │ walking: velocity or goal, obstacle settings                 ▲
     │ (SpotNavigationRequest)                                      │ safe body velocity
     ▼                                                              │ (v_x, v_y, v_rot)
LocalPlannerSystem ─────────────────────────────────────────────────┘
     │
     └─▶ SpotNavigationStatus (moving / near / at goal / blocked)

SpotApiSystem ──SpotArmCommand──▶ arm plugin (KinematicArmSystem)
```

The order within each simulation step comes from gz-sim system priorities:
SpotApiSystem (−100) writes the commands, LocalPlannerSystem (−50) refines the
walking velocity, then the base and arm plugins (default 0) read them.
SpotApiSystem reads the result back from Gazebo itself: body pose, joint
positions and velocities. Any plugin that moves the model works.

## Base: `KinematicBaseSystem`

- **No physics.** Every step it sets the model's pose directly. It integrates
  x, y and yaw from the commanded velocity (with acceleration ramps), and
  takes the height from leg IK on **flat ground at z = 0**.
- **The legs are animation.** The trot moves them; the body doesn't depend
  on where the feet are.
- **Gravity and collisions are off** for all robot links (collide bitmask 0),
  so the robot **walks through anything**. A forced pose can't be stopped by
  physics, and with collisions on it would only shove movable objects around.
- **Sit and stand** are scripted joint interpolations (1.5 s); losing motor
  power collapses the robot within 0.4 s.

**Planned replacement:** an RL locomotion policy, `RlBaseSystem` (see
`docs/handoff/rl_locomotion.md`). It will be torque-driven with gravity and
collisions on, so walls stop it physically and it walks on real terrain. It
reads the same `SpotLocomotionCommand`, so nothing else changes.

## Arm: `KinematicArmSystem`

- Sets the arm and gripper joints directly, moving toward the firmware's
  setpoint at a speed limit. No physics, no collisions with the world or the
  body.
- The firmware counts a move as done once every joint is within 0.01 rad of
  its setpoint (`kArmReachedTolerance`).

## Local planner: `LocalPlannerSystem`

The planner handles walking, and only walking. While the
`SpotLocomotionCommand`'s mode is "velocity", it replaces the velocity with a
planned one. Sit, stand, posture and power never pass through it. It runs
only when it has something to do:

| Firmware request | Planner |
|---|---|
| Velocity command (`cmd_vel`, `synchro_velocity`), avoidance on (the default) | Follows the commanded velocity and heading, keeps `obstacle_avoidance_padding` (default 0.1 m) from obstacles, slides along them, stops in front of them |
| Velocity command with `disable_vision_body_obstacle_avoidance` | Not involved: the velocity goes to the base unchanged |
| `se2_trajectory_request` (the driver's `trajectory` action) | Walks to the trajectory's last point around obstacles. Reports moving / near / at goal / blocked, which the firmware turns into `SE2TrajectoryCommand` feedback |
| Anything else (stand, sit, stop, ...) | Idle |

**The map** is a robot-centred 2.5D height map: 8 × 8 m at 5 cm, rolling
with the robot.
- It is filled from **ground truth**: the collision shapes of every other
  model in the world (boxes, spheres, cylinders, capsules, planes; meshes and
  heightmaps are not supported yet).
- **Static models** are drawn once per cell. **Movable ones** are redrawn
  every cycle (20 Hz), so moving obstacles are simply part of the map. The
  planner reacts to them by replanning, without predicting where they go.
- Anything more than 0.8 m above the robot's feet is ignored, so the robot
  walks under tables.

**From the map** the planner derives:
- **traversability:** steps up to 0.18 m and slopes up to 25° are walkable,
  matching the RL policy's training limits;
- **a distance field** to the nearest obstacle;
- **for goals, a cost-to-go field:** the cost of the cheapest way to the goal
  from every cell, which knows the way around walls and out of dead ends.

**MPPI** (`spot_sim_planning::Mppi`) samples 1000 body-velocity sequences
over 2 s every 50 ms, scores them on these fields, and averages them.
- **A goal counts as reached** within 0.1 m and 0.1 rad.
- **A goal is blocked** when it's unreachable, or when there has been no
  progress for 1.5 s. The robot keeps trying and moves on if the way clears.

**Timing:** each plan starts from the state at simulation time t and is
applied at t + 50 ms. If it isn't ready, the simulation waits, so results
don't depend on CPU speed or the real-time factor.

**Code:** the algorithms live in `src/spot_sim_planning`, which is pure C++
and unit-tested. The Gazebo adapter is `src/spot_sim_gz/src/local_planner_system.cpp`.
Turn the planner off with `sim.launch.py planner:=false`, and show its chosen
path in the Gazebo GUI with `planner_markers:=true`.

## What collides with what

| | Today (kinematic base) | With the RL base |
|---|---|---|
| Robot body / legs vs. world | Nothing; the planner keeps the robot clear when avoidance is on | Physical contact |
| Arm vs. world, arm vs. body | Nothing | Nothing (the arm stays kinematic) |
| Movable objects | Seen by the planner; the robot never touches them | Pushed by physics |
| Terrain | Flat ground only (the base ignores it; the planner rates it) | Real terrain, within the policy's training limits |

## Replacing a plugin

- **Base:** read `SpotLocomotionCommand` and write `SpotLocomotionStatus`
  (state, reached sequence) every step. Move the model however you like.
  Load it with `sim.launch.py base_plugin:=<Name>`.
- **Arm:** read `SpotArmCommand` and move the arm and gripper joints. Load it
  with `arm_plugin:=<Name>`.
- **Planner:**
  - read `SpotNavigationRequest`;
  - rewrite only the velocity of `SpotLocomotionCommand`, and only in
    velocity mode;
  - write `SpotNavigationStatus`;
  - run between SpotApiSystem and the base (priority between −100 and 0).

  Without a planner, walking to goals is unsupported, and velocity commands
  reach the base unchanged.
