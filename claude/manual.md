# Manual: Webots + Nav2 for the Agrobot

What changed since commit `f4a0384 agrobot webots v1`, and why. The original plan and
status log are in [instructions.md](instructions.md). This file explains the reasoning
behind the result.

## Goal

Before this work, `agrobot_webots` only listened to `/cmd_vel`. It published no odometry,
TF or sensor data, so nothing could navigate. The goal was the smallest setup where you
**click a point in RViz and the robot drives there, going around obstacles**.

Scope decision: **no map**. No SLAM, AMCL or map_server. Nav2 runs with `odom` as its
global frame, and both costmaps are rolling windows built from the lidar. This is the
minimum Nav2 needs to accept a `NavigateToPose` goal. A `map` frame can be added later
without changing the rest.

## What was done, file by file

### 1. Driver plugin — [agrobot_driver.py](../agrobot_webots/agrobot_webots/agrobot_driver.py)

The driver is a **webots_ros2_driver Python plugin**. `webots_ros2_driver` loads it from
the URDF `<plugin>` tag and calls `init()` once, then `step()` every simulation step. It now
does five jobs.

**a) Motors (as before, plus a separation scale).** `cmd_vel` → left/right wheel speeds
with differential-drive kinematics. Wheel separation is now
`wheelSeparation × wheelSeparationScale`.
*Why:* a skid-steer robot doesn't turn like an ideal differential drive. The wheels drag
sideways, so the robot turns less than the geometry predicts. Treating the track as wider
(scale 1.33, tuned in simulation) makes commanded and actual turn rates match. All three
values are now URDF properties, not hard-coded constants.

**b) Odometry `/odom` + TF `odom → base_link`.** The plugin reads the four wheel
`PositionSensor`s (`*_wheel_joint_sensor`), averages each side, and integrates x and y with
the midpoint method.
*Why:* Nav2 can't run without `odom → base_link`, and the robot had no source for it.

**c) Yaw comes from the IMU, not the wheels.** An `InertialUnit` (`imu`) was added to the
PROTO. When it is present, `d_yaw` comes from the IMU (with its initial offset removed)
instead of `(d_right - d_left) / separation`.
*Why:* wheel-based yaw was the worst error source. Skid-steer slip makes it wrong after
every turn, and one degree of yaw error grows into a large position error over distance.
The IMU yaw matched Webots ground truth. Distance still comes from the wheels.

**d) `/scan` is published by the plugin.** It reads `front_lidar` directly
(`getRangeImage()`) and publishes a `LaserScan` every 3 simulation steps.
*Why:* this was the hardest bug. webots_ros2_driver has its own lidar publisher
(`Ros2Lidar`), but it stamped every scan with **time 0**, because its node clock never
advanced. Nav2's costmap TF message filter **silently dropped** every scan, so the
costmaps stayed empty and the robot drove into crates without any error. Publishing from
the plugin, with the same `robot.getTime()` clock as the odometry, fixed it. For the same
reason, the URDF now disables the driver's own lidar publisher
(`<device reference="front_lidar"><ros><enabled>false</enabled>`).
Details:
- Webots orders rays left to right (+fov/2 → -fov/2). ROS expects counter-clockwise from
  `angle_min`, so the array is reversed.
- The first read is delayed by 2 lidar periods. Reading the range image before the first
  sample exists crashed the controller.
- QoS is `sensor_data` (best effort), the standard for scans and what the costmaps expect.

**e) `/joint_states` for the wheels.** *Why:* `robot_state_publisher` only publishes TF
for continuous joints when it receives their positions. Without this, the wheel frames are
missing and RViz shows the RobotModel with errors.

**Timestamps:** everything is stamped with `robot.getTime()` (Webots sim time), so the
plugin doesn't depend on its private node receiving `/clock`.

**Kept on purpose:** `rclpy.init()` + a private node inside `init()`. The first plan said
to remove them, but this is the official webots_ros2 Python plugin pattern. `webots_node`
only exposes `.robot`, not a ROS node.

**Removed:** `controllers/agrobot_driver/agrobot_driver.py`, a standalone Webots
controller that duplicated the plugin. The world uses `controller "<extern>"`, so only the
plugin runs, and the duplicate just caused confusion. Its install rule in `setup.py` was
removed too.

### 2. Robot model — [agrobot.proto](../agrobot_webots/protos/agrobot.proto) and [agrobot.urdf](../agrobot_description/urdf/agrobot.urdf)

**New low front lidar** (`front_lidar` in the PROTO, `front_lidar_link` in the URDF) at
(0.65, 0, -0.15) from `base_link`: 3.0 rad (~172°) FOV, 300 rays, 0.1–8 m.
*Why:* the existing `lidar_link` sits about 0.57 m above the floor. The crates are 0.3 m
tall and the boxes 0.2 m, so a lidar there sees **nothing** in this arena. The new one is
about 0.14 m above the floor. It sits ahead of the wheels, and its FOV stays under 180°, so
it never hits the robot's own wheels.
*Why a new link instead of moving `lidar_link`:* Gazebo uses the same URDF and its
`gpu_lidar` is attached to `lidar_link`. Leaving that link alone keeps the Gazebo setup
unchanged.

**New `InertialUnit`** in the PROTO (see 1c). The URDF already had an `imu_link` for
Gazebo. The Webots device sits on the robot's root body.

**The URDF `<webots>` block** now passes `wheelRadius`, `wheelSeparation` and
`wheelSeparationScale` to the plugin, and disables the driver's built-in lidar publisher
(see 1d).

### 3. World — [obstacle_arena.wbt](../agrobot_webots/worlds/obstacle_arena.wbt)

- **Four walls** at ±2.55 m (0.4 m tall, 0.1 m thick) around the 5 × 5 m floor.
  *Why:* without walls the robot could drive off the floor, and the lidar had nothing to
  see at the edges.
- **Spawn moved from (0, 0) to (-1.0, -0.3).**
  *Why:* at (0, 0), the robot's 1.2 × 1.12 m footprint overlapped `plastic crate(4)` at
  (0.19, 0.37). It spawned inside an obstacle, and the costmap marked the robot's own
  position as lethal.

Side effect: `odom` starts at the spawn, so goals are relative to (-1.0, -0.3) in world
coordinates.

### 4. Launch — [simulation.launch.py](../agrobot_webots/launch/simulation.launch.py)

New nodes alongside Webots and the driver:
- **`robot_state_publisher`** with the URDF contents and `use_sim_time`. It publishes the
  static TF `base_link → wheels / front_lidar_link / imu_link / ...`. *Why:* Nav2 has to
  transform `/scan` from `front_lidar_link` into `odom`.
- **`navigation.launch.py`**, included when `nav:=true` (default).
- **RViz** with `rviz/nav.rviz`, when `rviz:=true` (default).
- **Shutdown handler:** when the Webots process exits, the whole launch shuts down.
  *Why:* otherwise closing Webots leaves Nav2 and RViz running with no simulation behind
  them.

### 5. Nav2 launch — [navigation.launch.py](../agrobot_webots/launch/navigation.launch.py) (new)

Starts only `controller_server`, `smoother_server`, `planner_server`, `behavior_server`,
`velocity_smoother`, `bt_navigator` and one `lifecycle_manager` (autostart).

*Why not `nav2_bringup/navigation_launch.py`:* in Jazzy it also starts `docking_server`,
`collision_monitor` and `waypoint_follower`. `docking_server` fails to configure without
dock plugins, which aborts the lifecycle manager, so nothing ever becomes active. Writing
our own launch file avoids that and keeps the node set minimal.

Velocity chain (the standard Nav2 one):
`controller_server` / `behavior_server` → `/cmd_vel_nav` → `velocity_smoother` → `/cmd_vel` → plugin.

### 6. Nav2 parameters — [nav2_params.yaml](../agrobot_webots/config/nav2_params.yaml) (new)

| Choice | Why |
|---|---|
| `global_frame: odom` everywhere, rolling-window costmaps, no static layer | No map in this scope |
| Regulated Pure Pursuit, `use_rotate_to_heading`, no reversing | Simple and predictable path follower that suits a skid-steer (it can turn in place) |
| NavFn planner with `allow_unknown` | Most of the rolling global costmap is unknown at the start |
| Footprint 1.2 × 1.12 m | The wheels stick out past the chassis: y = ±0.48 ± 0.07, x = ±0.35 ± 0.22. The first estimate of 1.2 × 0.8 would let the wheels hit obstacles |
| Inflation radius 0.7 m | Must be at least the inscribed radius (~0.56 m) or the planner routes too close to obstacles |
| `min_obstacle_height: -1.0` at **both** layer and source level | The lidar is 0.15 m **below** `base_link`, so its points have z < 0 in `odom`. The default (0) filters them all out. Setting it only on the `scan` source was not enough: clearing worked but nothing was marked. The layer-level value is a separate filter |
| Goal tolerance 0.15 m / 0.3 rad, `stateful` goal checker | Reachable with this odometry quality. Once xy is reached, the robot only rotates to the final heading |
| Velocity limits 0.5 m/s, 1.0 rad/s | Moderate speeds for a 1.2 m robot in a 5 m arena |

### 7. RViz config — [nav.rviz](../agrobot_webots/rviz/nav.rviz) (new)

Fixed frame `odom`, top-down view. Displays: Grid, RobotModel (from
`/robot_description`), TF, LaserScan, global and local costmaps, footprint, global plan,
odometry arrows. The **Nav2 Goal** tool (SetGoal) publishes to `/goal_pose`, and
`bt_navigator` turns that into a `NavigateToPose` goal.

### 8. Packaging — [setup.py](../agrobot_webots/setup.py) and [package.xml](../agrobot_webots/package.xml)

- `setup.py` installs `config/*.yaml` and `rviz/*.rviz`, and no longer installs the
  removed controller.
- `package.xml` adds the new runtime dependencies: `builtin_interfaces`, `launch_ros`,
  `nav_msgs`, `navigation2`, `robot_state_publisher`, `rviz2`, `tf2_ros`. `sensor_msgs` is
  missing even though the driver imports it. It comes in through `navigation2`, but it
  should be listed explicitly.

## How it was verified (2026-10-02)

In Webots, end to end:
1. Teleop with `/cmd_vel`: `/odom` follows the robot. After calibrating the separation
   scale and switching to IMU yaw, odometry yaw matched Webots ground truth.
2. `/scan` arrives at about 10 Hz with sim-time stamps, and the crates appear in both
   costmaps.
3. All lifecycle nodes become `active`.
4. Nav2 goals to (2.2, 1.2) and back to (0, 0) in `odom`: **both SUCCEEDED**. The path went
   around `plastic crate(4)` and no object was pushed.

## Limits and what to do next

- **Position drift:** about 0.3 m true error per goal and about 0.4 m after a round trip.
  The cause is sideways slip in turns, which neither the encoders nor the IMU yaw can see.
  Next step: a `robot_localization` EKF (wheel odom + IMU), then GPS or SLAM/AMCL to get a
  real `map` frame. When that exists, change `global_frame` to `map` in the global costmap
  and `bt_navigator`, and add a static layer if you use a map.
- **Testing gotcha:** killing `ros2 launch` with SIGKILL leaves orphan Nav2 nodes. Their
  `lifecycle_manager` deactivates the next run's nodes. Run `pkill -9 -f nav2_` before
  relaunching.
- The work is **uncommitted**. Only the deletion of the old controller is staged.

## Registro de alterações

### 2026-10-06 — Remoção do `agrobot.urdf.xacro` antigo

**O que mudou**
- Apagado `agrobot_description/urdf/agrobot.urdf.xacro`.
- `README.md`: a linha do layout do repositório deixou de citar o `.xacro`.

**Por quê.** Havia a suspeita de que o plugin de tração diferencial usava medidas de roda
erradas (raio 0,22 m e separação 0,96 m contra 0,165 m e 0,86 m "do modelo"). A conferência
mostrou que o plugin está certo:
- Todos os launches (`spawn_agrobot`, `agrobot_gazebo/simulation`,
  `stability_simulation`, `agrobot_webots/simulation`) carregam `agrobot.urdf`.
- Em `agrobot.urdf` as rodas têm raio 0,22 m e as juntas ficam em y = ±0,48 m
  (separação 0,96 m). O plugin `gz-sim-diff-drive-system`, o `<plugin>` do Webots e o
  `agrobot.proto` usam esses mesmos valores.
- Os valores 0,165 m e 0,86 m (y = ±0,43 m) só existiam no `.xacro`, um modelo antigo e
  simplificado, sem plugin, que nenhum launch carrega.

O `.xacro` foi removido para não gerar esse diagnóstico errado de novo. Nenhum parâmetro de
roda foi alterado. O `agrobot.urdf.backup` foi mantido.

**Se o `/odom` do Gazebo divergir na prática**, a causa provável é o skid-steer: o
`DiffDrive` calcula o giro pelas rodas, que deslizam lateralmente nas curvas. No Webots isso
já foi resolvido com o yaw do IMU e o `wheelSeparationScale` (seção 1).

**Validação.** O `CMakeLists.txt` instala o diretório `urdf/` inteiro, então nada depende
do arquivo pelo nome; `grep` no repositório não encontra outra referência ao `.xacro`.
O `colcon build` não foi executado porque o ambiente de nuvem não tem ROS 2 instalado.
