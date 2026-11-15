# Handoff: RL locomotion policy for spot-sim

**For:** an agent on a GPU machine (RTX 2070 SUPER, 8 GB VRAM, Turing, compute capability 7.5).
**From:** the spot-sim maintainer session, 2026-11-15.
**Goal:**
- Train a velocity-commanded locomotion policy for Spot that uses privileged simulator information (heightmap, true base velocity).
- Deploy it in spot-sim as a Gazebo plugin that replaces the kinematic base.

Read this whole document before starting. Where it says **verify**, check before relying on it.

---

## 1. Repositories

| What | URL | Notes |
|---|---|---|
| **spot-sim** (this project) | https://github.com/taDachs/spot-sim (ssh: `git@github.com:taDachs/spot-sim.git`) | Work on branch **`spot-sim`**; create your branch **`rl-base`** from it. Clone with submodules: `git clone --recurse-submodules -b spot-sim https://github.com/taDachs/spot-sim.git` |
| spot_ros2 (driver; submodule `spot_ros2/`) | https://github.com/rai-opensource/spot_ros2 | Pinned at `949feb7`. Only its robot description matters to you. |
| spot_description (submodule `spot_ros2/spot_description/`) | https://github.com/rai-opensource/spot_description | Pinned at `156d180`. **The robot model you must train on**: URDF/xacro + meshes. |
| spot_wrapper (submodule of spot_ros2) | https://github.com/rai-opensource/spot_wrapper | Not needed for training. |
| Isaac Lab (recommended trainer) | https://github.com/isaac-sim/IsaacLab | Height scanner, terrain curriculum, rsl_rl integration, a Spot velocity task to copy from |
| rsl_rl (PPO) | https://github.com/leggedrobotics/rsl_rl | |
| MuJoCo Playground (fallback trainer) | https://github.com/google-deepmind/mujoco_playground | MJX/JAX; its quadruped joystick tasks include Spot (**verify** the env name in the version you install) |
| MuJoCo Menagerie | https://github.com/google-deepmind/mujoco_menagerie | `boston_dynamics_spot` (a reference only; train on spot_description, see §3) |
| legged_gym (second fallback) | https://github.com/leggedrobotics/legged_gym | Isaac Gym Preview 4 runs well on 8 GB Turing, but it's deprecated (Python 3.8) |
| ONNX Runtime | https://github.com/microsoft/onnxruntime | C++ inference in the Gazebo plugin (prebuilt Linux x64 release tarballs) |
| Gazebo Harmonic docs | https://gazebosim.org/docs/harmonic | gz-sim 8 C++ API: `gz::sim::Joint::SetForce`, `Link`, `Model` |

The training code goes in a **new repository** (suggested name: `spot-sim-rl`). Ask the user for the remote before you push. The trained policy files are small and get committed to spot-sim (§7).

---

## 2. Context: what spot-sim is and where your work plugs in

spot-sim simulates Boston Dynamics Spot in **Gazebo Harmonic** (gz-sim 8) on **ROS 2 Jazzy** (Docker image `spot-sim/sim`, Ubuntu 24.04). It serves the real Spot gRPC API, so the unmodified `spot_ros2` driver connects to it as if it were a robot. Everything runs in Docker; see `compose.yaml` and the comment at its top.

The robot is moved by a swappable **base plugin**:
- The Spot API side (`src/spot_sim_core`, called "firmware") turns API commands (stand, sit, velocity, body posture, power) into a plain `LocomotionCommand` struct.
- `SpotApiSystem` (`src/spot_sim_gz/src/spot_api_system.cpp`) writes the struct into a Gazebo component on the robot model every step.
- The base plugin reads it, moves the robot, and writes back a `LocomotionStatus`.
- `SpotApiSystem` reads the body pose, joint positions and velocities back from Gazebo itself, so any base works.

**Today's base is `KinematicBaseSystem`** (`src/spot_sim_gz/src/kinematic_base_system.cpp`):
- There is no physics: it sets the model pose directly each step and animates a trot.
- Gravity and collisions are off for the robot, so it walks through obstacles on flat ground (z = 0).

**Your plugin, `RlBaseSystem`, replaces it:**
- The robot becomes a physical, torque-controlled body: gravity and collisions on, joints driven by PD torques from your policy.
- The swap point already exists: the launch argument `base_plugin:=RlBaseSystem` (`src/spot_sim_bringup/launch/sim.launch.py`) is passed to `spot_sim.urdf.xacro`, which loads it as the model's plugin.

**In parallel**, the maintainer session builds a **local planner** and a **2.5D height map** of the robot's surroundings. The planner drives your policy through the same velocity commands, so it needs nothing from you. The height map is a shared interface, described in §6.4.

---

## 3. The robot model (the training model must match it)

Gazebo builds the robot from `src/spot_sim_description/urdf/spot_sim.urdf.xacro`. That file includes spot_description's `spot_macro.xacro` with `load_spot(arm=<ARM>, feet=false, gripperless=false, tf_prefix="", include_transmissions=false)`.

**Train on this exact URDF** (convert it to USD/MJCF with your trainer's importer), not on Isaac Lab's or Menagerie's own Spot assets. Masses, inertias, limits and collision geometry must match Gazebo.

To get the URDF the sim uses (the sim image has xacro and the packages installed):

```bash
docker compose build sim
docker run --rm spot-sim/sim:latest bash -lc \
  'xacro $(ros2 pkg prefix spot_sim_description)/share/spot_sim_description/urdf/spot_sim.urdf.xacro arm:=false cameras:=false' > spot.urdf
# arm:=true for the variant with the Spot Arm. Meshes: spot_ros2/spot_description/spot_description/meshes/
```

The image's entrypoint sources `/opt/ros/jazzy` and the workspace `/ws/install` (checked: this command prints the URDF).

Facts taken from spot_description:

| Item | Value |
|---|---|
| Body mass | 32.86 kg (link `body`) |
| Per leg | hip 1.68 kg, upper leg 2.34 kg, lower leg 0.35 kg |
| Arm + gripper (`ARM=true`) | about 7.0 kg in total, mounted at body (0.292, 0, 0.188) |
| Leg joints, in this order | `front_left_hip_x, front_left_hip_y, front_left_knee, front_right_hip_x, front_right_hip_y, front_right_knee, rear_left_hip_x, rear_left_hip_y, rear_left_knee, rear_right_hip_x, rear_right_hip_y, rear_right_knee` |
| hip_x limits | ±0.7854 rad, 45 Nm, 17.647 rad/s |
| hip_y limits | -0.8988 … 2.2951 rad, 45 Nm, 17.647 rad/s |
| knee limits | -2.7929 … about -0.25 rad (upper limit differs per leg: -0.2548, -0.2476, -0.2672, -0.2577), 115 Nm, 12.0 rad/s |
| Feet | no foot links (`feet=false`). Each lower leg has a collision sphere r = 0.036 at (0, 0, -0.3365) in the lower-leg frame. Other leg and body collisions are meshes (`meshes/base/collision/*.obj`). |
| Nominal stand | body origin 0.52 m above the ground, feet directly below the hips (`kNominalStandHeight`, `leg_kinematics.hpp`) |
| Sit pose | hip_y = 1.2, knee = -2.7, hip_x = 0 |
| Hip geometry | hip_x joints at (±0.29785, ±0.055, 0) in the body frame |

**With the arm (`ARM=true`):**
- `KinematicArmSystem` sets the arm's joint positions directly every step, so the arm moves rigidly relative to the body. Its mass and inertia still load the body.
- The arm swings between stowed (folded on the body) and extended forward (about 1 m reach), so the centre of mass shifts.
- **One policy must handle both variants:** randomize the payload and its centre of mass (§5).

---

## 4. Interface contract with spot-sim (do not change without the maintainer)

### 4.1 Input: `spot_sim::LocomotionCommand`

From `src/spot_sim_core/include/spot_sim_core/locomotion.hpp`, read from component `gz::sim::components::SpotLocomotionCommand` (`src/spot_sim_gz/include/spot_sim_gz/components.hpp`) on the model entity, every step:

| Field | Meaning |
|---|---|
| `motors_on` | false → the robot collapses: no torque, or light damping, legs fold. Real Spot sits down limp. |
| `mode` | `kSit`: sit down. `kStand`: stand still. `kVelocity`: walk at `v_*`. `kStop`: stop walking and keep the posture. |
| `sequence` | Goes up with every new command. Echo it in `LocomotionStatus::reached_sequence` once the command's target is reached. |
| `v_x, v_y, v_rot` | Body velocity target in the **gravity-aligned body (heading) frame**, [m/s, rad/s]. Already clamped to at most \|v_x\| ≤ 1.6, \|v_y\| ≤ 0.6, \|v_rot\| ≤ 1.2 (the API's defaults; clients may set lower limits). |
| `max_accel_x/y/rot` | How fast the commanded velocity may change [m/s², rad/s²]; default 2.0. Ramp the policy's command with these. |
| `body_height_offset`, `body_roll`, `body_pitch`, `body_yaw` | Body posture relative to the nominal stand, in the footprint frame. Limits: height ±0.15 m, roll and pitch ±0.4 rad, yaw ±0.5 rad. Phase 2 (§8). |

### 4.2 Output: `spot_sim::LocomotionStatus`

Written to component `SpotLocomotionStatus` every step:

| Field | Meaning |
|---|---|
| `state` | `kSitting`, `kTransitioning` (standing up, sitting down, collapsing), `kStanding` (up, not walking), `kMoving` (up and walking) |
| `reached_sequence` | Sequence of the last command whose target was reached. **Sit:** seated. **Stand / stop:** up, at the posture target. **Velocity:** up. The firmware's stand, sit and stop feedback depends on this. |

**Reference behaviour:** `KinematicBaseSystem::Status()` and `AdvancePosture()`.
- It stands up and sits down over 1.5 s, and collapses within 0.4 s when the motors go off.
- It walks only once fully standing.
- When sitting is commanded while walking, it first stops and levels the body.

**Falling** has no state yet. If the policy falls:
- report `kTransitioning`;
- stop tracking velocity;
- and propose an extension (`kFallen`) to the maintainer. Don't change the struct on your own: the firmware and the API's self-right command depend on it.

### 4.3 What the rest of the stack reads from Gazebo (keep it working)

- **Body link `body`:** its world pose. Odometry is relative to the boot pose.
- **Joint positions and velocities** of the 12 leg joints (names above), through `gz::sim::Joint`. Leave `EnablePositionCheck`/`EnableVelocityCheck` alone; `SpotApiSystem` enables them.
- **Lower-leg link world poses**, for foot positions. Foot contact is currently computed as "foot sphere less than 2 cm above z = 0", which is wrong on terrain; that's the maintainer's problem, so just mention it in your report.

### 4.4 Plugin mechanics

- **Base it on the existing plugin.** Copy `KinematicBaseSystem`'s skeleton (`Configure` finds the model and joints; `PreUpdate` reads the command and writes the status), and register it in `src/spot_sim_gz/CMakeLists.txt` the same way.
- **Don't disable gravity or collisions.** That code exists only in `KinematicBaseSystem`.
- **Torque control:** `Joint::SetForce(ecm, {tau})` per joint, every physics step. Physics runs at **1 ms** (`step_size` launch argument, default 0.001), and real-time factor 1 must hold on the user's CPU with cameras off.
- **The PD law and its rate must match training.** Either run PD at 1 kHz in Gazebo and train with the same, or hold the torque for the training PD period (for example 5 ms). Measure both in sim-to-sim tests.
- **Policy rate:** 50 Hz is suggested (decimation 20 at 1 ms). Keep the timing in the spec file (§7).
- **Inference on the CPU.** The sim container has no CUDA, and an MLP of this size is trivial. Options:
  - ONNX Runtime C++, as a prebuilt release tarball downloaded in `docker/sim.Dockerfile` (**verify** whether Ubuntu 24.04's apt has a usable `libonnxruntime-dev` first);
  - or a dependency-free MLP with Eigen, from exported weights. Choose one and document why.
- **Observations available in Gazebo** (privileged, since this is simulation; no estimation needed):
  - body pose and twist: `Link::WorldPose`, `WorldLinearVelocity`, `WorldAngularVelocity` (enable velocity checks on the link);
  - joint states;
  - contacts, through contact sensors or the physics contact component, only if your observations need them;
  - the heightmap, from §6.4.
- **Physics engine:** gz Harmonic defaults to DART. The `bullet-featherstone` plugin is available too.
  - Pick whichever reproduces training best and set it in the world files (`<plugin filename="gz-sim-physics-system">` takes an `<engine><filename>` element).
  - Set the ground friction explicitly in the worlds (`<surface><friction>`). Document the choice.

---

## 5. Training specification (suggested; adjust with reasons)

**Framework:**
- **Isaac Lab + rsl_rl.** It has the height scanner, terrain generator and curriculum, and a Spot locomotion task to copy rewards from (`Isaac-Velocity-Flat-Spot-v0`; **verify** the name and whether there's a rough-terrain variant).
- **8 GB VRAM is at the limit** for Isaac Sim, so do a feasibility spike first: does it start headless, and how many environments fit (aim for 2048–4096)?
- **If it doesn't run well**, fall back to MuJoCo Playground (MJX, hfield terrain) or legged_gym (Isaac Gym Preview 4).
- Record the decision and the measured throughput in your report.

**Observations (all privileged, available identically in Gazebo):**
- base linear velocity (body frame), base angular velocity (body frame), projected gravity;
- command (v_x, v_y, v_rot);
- joint positions minus their default, joint velocities;
- last action;
- **heightmap.**
  - Suggested grid: 1.6 m × 1.0 m around the body, 0.1 m spacing (17 × 11 = 187 points), centred on the base, yaw-aligned with the heading.
  - Values: base height minus terrain height minus 0.5, clipped to ±1 m.
  - The **exact pattern is part of the contract**: put it in the spec file, because the Gazebo side samples the same points.

**Actions:**
- 12 joint position offsets: target = default stand pose + scale · action (scale about 0.2–0.25).
- PD gains: start from Isaac Lab's Spot actuator config (**verify** its knee actuator model; the knee is 115 Nm, the hips 45 Nm).
- Clip torques to the URDF effort limits.

**Command ranges:**
- v_x from -1.0 to 1.6, v_y ±0.6, v_rot ±1.2.
- Include a share of exact-zero commands: at zero command the robot **stands still** without stepping in place.

**Terrain curriculum:**
- flat, random rough (±5 cm), slopes up to 20°, steps up and down to 15 cm, stairs (17 cm risers, 28 cm treads), discrete obstacles.
- Start flat and get harder with success.

**Domain randomization** (the target is sim-to-sim into Gazebo DART, a different contact model, so randomize generously):
- ground friction 0.4–1.25;
- payload 0–8 kg at the arm mount, with centre-of-mass offsets of ±0.15 m (covers ARM=true);
- base mass ±3 kg;
- PD gains ±20 %, motor strength ±10 %;
- action latency 0–20 ms;
- random pushes;
- observation noise;
- joint friction and damping.

**Rewards:** the usual velocity-tracking set (linear and angular tracking, z velocity, orientation, torques, action rate, feet air time, collisions on body and legs, joint limits). **Plus** a strong standing-still term at zero command, and a base height term near 0.52 m.

**Episode reset:** start sitting or standing, so the policy also learns to start walking from a stand. Sit and stand transitions themselves are scripted (§6.2).

---

## 6. Gazebo deployment: `RlBaseSystem`

### 6.1 Policy control

- In `kStand`, `kStop` and `kVelocity` (once standing), the policy runs at 50 Hz.
- Its command is the ramped `v_*`, zero in `kStand` and `kStop`.
- PD torques are applied every physics step.

### 6.2 Scripted transitions

These don't use the policy:
- **Stand up:** PD interpolation from the sit pose to the stand pose over 1.5 s, with feet on the ground. Then the policy takes over with a zero command.
- **Sit down:** stop (zero command until the base velocity is about 0), then PD interpolation to the sit pose over 1.5 s. Then `kSitting`.
- **Motors off:** zero torque plus small joint damping. The body falls and the legs fold; that's a collapse, as on the real robot.
- **The status logic** mirrors `KinematicBaseSystem::Status()`.

### 6.3 Body posture (phase 2)

- Static posture while standing still is a PD stance controller: IK to joint targets with the feet fixed.
- Reuse `spot_sim::Stance` (`src/spot_sim_core/include/spot_sim_core/stance.hpp`) and `LegKinematics`. They already compute leg angles for a posture with the feet fixed.
- Height offset and pitch while walking would need to be policy inputs: phase 2.

### 6.4 Heightmap source in Gazebo (shared with the planner)

- **You define the interface** in `src/spot_sim_core/include/spot_sim_core/height_field.hpp`, for example:

  ```cpp
  namespace spot_sim {
  /// Terrain height [m] at a world (x, y); implementations must be cheap (called ~200x per policy tick).
  class HeightField { public: virtual ~HeightField() = default; virtual double HeightAt(double x, double y) const = 0; };
  class FlatGround final : public HeightField { public: double HeightAt(double, double) const override { return 0.0; } };
  }
  ```

- **You also provide a sampler** that turns a body pose into the policy's heightmap observation, following the spec's pattern.
- **The maintainer will implement a Gazebo-backed `HeightField`** (a 2.5D map from the world's collision geometry) for the planner, and hand it to your plugin.
- **Until then:**
  - use `FlatGround` for the `empty` and `yard` worlds;
  - for terrain tests, use your own quick `HeightField` from the test world's known geometry, or from a Gazebo `<heightmap>` image. Keep it out of the public interface.
- Agree on the coordinate convention (world frame, height of the terrain's top surface) and keep it in the header comment.

---

## 7. Deliverables

1. **The training repository** (`spot-sim-rl`; ask the user for its remote):
   - environment and config, rewards, terrain and randomization;
   - training and export scripts;
   - a README with exact reproduction steps, seeds and training time on the 2070 SUPER.
2. **The policy, in spot-sim** at `src/spot_sim_gz/policies/<name>/`:
   - `policy.onnx` (or the weights for the Eigen MLP);
   - `policy_spec.yaml`, the single source of truth for deployment:
     - observation layout, order, scales and clipping;
     - the heightmap pattern;
     - default joint positions;
     - action scale;
     - PD gains and torque limits;
     - control and PD rates, and latency assumptions;
     - command ranges;
     - the URDF variant it was trained on, including the spot_description commit.
3. **`RlBaseSystem`** in `src/spot_sim_gz/src/rl_base_system.cpp`, plus CMake.
   - Add a `BASE_PLUGIN` setting to `compose.yaml`, passed as `base_plugin:=${BASE_PLUGIN:-KinematicBaseSystem}`, and document it in the compose header.
   - Add the inference dependency to `docker/sim.Dockerfile`.
   - Add `height_field.hpp` (§6.4) with unit tests in `src/spot_sim_core/test/`.
   - A terrain test world, `src/spot_sim_bringup/worlds/terrain.sdf` (slopes, steps, stairs, rough patch), written in the style of `yard.sdf` (template placeholders).
4. **`docs/rl_base.md`:**
   - how it works and how to retrain or replace the policy;
   - the physics engine and contact settings chosen;
   - the sim-to-sim results (below);
   - known gaps.
   - Add rows to `docs/limitations.md` for what's still missing (posture while walking, falls, ...).
5. **All on branch `rl-base`, as small commits.**
   - Don't touch `spot_sim_core` beyond `height_field.*`; the planner work happens there in parallel.
   - Don't touch the firmware or API code. Contract changes are proposals to the maintainer.

---

## 8. Acceptance criteria (measured in Gazebo, not only in the trainer)

- **Velocity tracking (flat ground, `WORLD=empty`):**
  - on a command grid of v_x ∈ {-1, -0.5, 0.5, 1, 1.5}, v_y ∈ {-0.5, 0.5}, v_rot ∈ {-1, 1} (singly and combined), the mean absolute tracking error is under 0.1 m/s and under 0.15 rad/s, measured after 1 s of settling;
  - report a table.
- **Standing still:** at zero command, drift under 2 cm per 10 s, no stepping in place, body height 0.52 ± 0.02 m.
- **Robustness:** 30 min of random commands (changed every 2–5 s), on flat ground and in `terrain.sdf` (slopes ≤ 15°, steps ≤ 12 cm, stairs): zero falls. Also with `ARM=true`, the arm cycling between stow and ready.
- **Transitions:** sit → stand → walk → stop → sit, repeated 20 times. `reached_sequence` reports every transition. No falls, no jumps.
- **Motors off while walking:** the robot collapses, and the plugin reports `kTransitioning`, then `kSitting`.
- **Real time:** RTF ≥ 1.0 headless at a 1 ms step, cameras off, on the user's machine. Report CPU load.
- **The existing e2e suite** (`docker compose --profile test run --rm e2e`) passes with `BASE_PLUGIN=RlBaseSystem`:
  - in both `ARM=true IMAGES=true POINT_CLOUDS=true WORLD=yard` and the defaults;
  - some tests assume the kinematic base's exact behaviour, such as the 1 cm body-height tolerance in `tests/e2e/robot_setup.py` (`wait_for_body_height`) and the exact odometry distances in `test_cmd_vel.py`;
  - you may relax those tolerances only when the suite runs with `RlBaseSystem`, and must list each change with its reason;
  - the kinematic base must stay the default, and green.
- **Phase 2 (optional, after the above):** body height and pitch while walking as policy inputs.

---

## 9. How to run spot-sim (quick reference)

```bash
git clone --recurse-submodules -b spot-sim https://github.com/taDachs/spot-sim.git && cd spot-sim
docker compose build                       # sim (Jazzy + Harmonic) and driver (Humble) images
docker compose up -d                       # headless sim + driver; WORLD=yard, ARM=true, ... see compose.yaml header
docker compose --profile test run --rm e2e # end-to-end tests (pytest) against the running stack
docker compose --profile gui up            # Gazebo GUI + RViz on your display
docker compose down
```

**Unit tests and builds** run inside the sim image. The workspace layout is in `docker/sim.Dockerfile`. A quick loop:
1. mount the repository's `src` into the container;
2. `colcon build`;
3. `SPOT_SIM_TEST_CERTS_DIR=/certs ./build/spot_sim_core/spot_sim_core_test`.

**Driving the robot by hand**, once the stack is up, through the driver's ROS interface (namespace `/spot`):
1. call the services `claim`, `power_on`, `stand`;
2. publish `geometry_msgs/Twist` on `/spot/cmd_vel`, which becomes a `kVelocity` command;
3. call `sit`, `power_off`, `release`.

The helpers are in `tests/e2e/robot_setup.py` and `tests/e2e/driving.py`.

**Further reading in the repository:**
- `docs/limitations.md`: what isn't simulated;
- the header comments in `kinematic_base_system.cpp`, `spot_api_system.cpp` and `components.hpp`: the plugin contract.

---

## 10. Open points to raise with the user or maintainer

- The remote for the training repository.
- Whether to store the policy files in git LFS (probably unnecessary: an MLP is well under 5 MB).
- The `kFallen` / self-right extension to `LocomotionState` if falls happen in practice.
- The heightmap pattern, once fixed, so the planner's 2.5D map can be sampled the same way.
