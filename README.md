# ros2_gazebo_mestrado — Agrobot simulation

ROS 2 workspace for **Agrobot**, a 4-wheel skid-steer agricultural robot, simulated in
**Webots** (main path, with Nav2 navigation) and **Gazebo Harmonic** (earlier path,
reactive obstacle avoidance and stability tests).

- ROS 2 **Jazzy** (Ubuntu 24.04, running under WSL2)
- Webots **R2025a**, installed on Windows (`WEBOTS_HOME=/mnt/c/Program Files/Webots`)
  and driven from WSL by `webots_ros2_driver` 2025.0.0
- Nav2 (`ros-jazzy-navigation2` 1.3.x)

## Current state

| Feature | Simulator | Status |
|---|---|---|
| Drive the robot with `/cmd_vel` | Webots | Works |
| Wheel + IMU odometry (`/odom`, TF `odom → base_link`) | Webots | Works |
| Low front lidar (`/scan`) that sees the crates and boxes | Webots | Works |
| Nav2: click a goal in RViz and the robot drives there around obstacles | Webots | Works, tested on 2026-10-02 |
| Goal from RViz without Nav2 or map: odometry only, avoids what is in front (`goal_navigator`) | Webots | New on 2026-10-06. Run in Webots R2025a on Linux (headless) on 2026-10-09: reaches the arena goals, but doesn't get into the crop-field lanes (see [Plantation field](#plantation-field-world)). Not yet on Windows |
| `robot_localization` EKF (wheels + IMU) as the `/odom` source, `ekf:=true` | Webots | New on 2026-10-07. Run in Webots R2025a on Linux (headless) with Nav2 on 2026-10-09 (crop-field route, with `slam:=true`), not yet on Windows |
| `/clock` from Webots sim time | Webots | New on 2026-10-07 |
| Lidar SLAM (`slam_toolbox`): `/map`, `map → odom`, and a `map`-frame EKF whose covariance ellipse shrinks where the lidar pins the pose, `slam:=true` | Webots | New on 2026-10-08. Run in Webots R2025a on Linux (headless) with Nav2 on 2026-10-09 (crop-field route); the ellipse wasn't checked in RViz, not yet on Windows |
| Crop-field test world (`plantation_field.wbt`): 6 plant rows, uneven ground, clods, stones and branches, `world:=plantation_field.wbt` | Webots | New on 2026-10-09, tested in Webots R2025a on Linux (headless) with the real driver, Nav2 and slam_toolbox, not yet on Windows. See [Plantation field](#plantation-field-world) |
| Olive-grove test world (`olive_grove.wbt`), based on a traditional grove on the Douro Internacional plateau: 7 m tree grid on a 4–5% slope, uneven stony red soil, pruning piles, stone heaps and rocks in the lanes, vineyard, dirt road, `world:=olive_grove.wbt` | Webots | New on 2026-11-12, tested in Webots R2025a on Linux (headless) with Nav2 and RTAB-Map, not yet on Windows. See [Olive grove](#olive-grove-world) |
| RGB-D camera for VSLAM (`/vslam_camera/...`: color, registered depth, `camera_info`, one stamp), `camera:=false` turns it off | Webots | New on 2026-10-20, tested in Webots R2025a on Linux (headless), measured with the `AgrobotDriver` code run outside `webots_ros2_driver`; on 2026-10-21 also started with `webots_ros2_driver` itself (topics published, none with `camera:=false`). Not yet in RViz or on Windows ([details](claude/manual.md#3-o-robô-no-webots)) |
| RGB data acquisition camera for crop monitoring (`/data_camera/image_raw`, `/data_camera/camera_info`), looking left at the plant row, 1280 × 960 about once per second, off by default, `data_camera:=true` | Webots | New on 2026-11-12. **Not run in Webots yet** (not even on Linux): only the geometry and the driver logic were checked, outside the simulator ([details](claude/manual.md#6-câmera-rgb-de-aquisição-de-dados-data_cameratrue)) |
| VSLAM with RTAB-Map (camera + lidar check): TF `map → odom`, `/rtabmap/map`, mapping and localization on a saved map, `vslam:=true` (needs `ekf:=true`) | Webots | New on 2026-10-21, tested in Webots R2025a on Linux (headless) on the crop field and the arena, without Nav2, with the driver code run outside `webots_ros2_driver` (and 40 s with `webots_ros2_driver` itself); not yet in RViz or on Windows. See [VSLAM](#vslam-rtab-map) |
| Waypoint missions: a YAML list of waypoints driven in sequence with Nav2 `FollowWaypoints`, a 3 s stop at each one, markers in RViz and a CSV log with the estimated and true pose, `waypoints:=<case>`, or clicked in RViz with `waypoints:=rviz` | Webots | New on 2026-10-26. Three study cases for comparing EKF, SLAM and VSLAM. Run in Webots R2025a on Linux (headless), not yet on Windows. See [Waypoint missions](#waypoint-missions) |
| Ground truth from the Webots Supervisor (`/ground_truth/odom`), for measuring errors only | Webots | New on 2026-10-26 |
| Nav2 plans in `map` when SLAM is on (`slam:=true` or `vslam:=true`); local costmap and controller stay in `odom` | Webots | New on 2026-10-26 |
| AMCL / GPS | — | Not done. VSLAM study: [claude/estudo_vslam.md](claude/estudo_vslam.md) |
| `cmd_vel` / `odom` / `scan` bridge | Gazebo | Works (from the earlier commits) |
| Reactive obstacle avoidance node | Gazebo | Code exists, but see [Known issues](#known-issues) |
| Roll/pitch stability monitor | Gazebo | Code exists, no entry point yet |

The Webots + Nav2 work is **not committed yet** (see `git status`).

## Repository layout

```
agrobot_description/     URDF, meshes. Shared by Gazebo and Webots
  urdf/agrobot.urdf        the URDF actually used (agrobot.urdf.backup is older)
  launch/spawn_agrobot.launch.py   spawns the robot into an already running Gazebo
agrobot_webots/          Webots simulation + Nav2 (ament_python)
  agrobot_webots/agrobot_driver.py   webots_ros2 plugin: motors, odom, TF, scan, joint_states
  protos/agrobot.proto     Webots robot (generated from the URDF, plus IMU and lidar)
  worlds/obstacle_arena.wbt  5 x 5 m arena, walls, 4 crates, 3 boxes
  worlds/plantation_field.wbt  crop field on uneven ground (generated, don't edit by hand)
  protos/CropPlant.proto   corn-like plant for the field (only the stalk collides)
  scripts/generate_plantation_field.py  generates plantation_field.wbt (fixed seed)
  worlds/olive_grove.wbt   traditional olive grove on a slope (generated, don't edit by hand)
  protos/OliveTree.proto   olive tree for the grove (only the trunk collides)
  scripts/generate_olive_grove.py  generates olive_grove.wbt (fixed seed)
  scripts/summarize_missions.py  table of the waypoint mission CSVs (one row per run)
  scripts/compare_methods.py  same CSVs, one row per case and mode (mean of the runs)
  launch/simulation.launch.py  Webots + driver + robot_state_publisher + Nav2 + RViz
  launch/navigation.launch.py  only the Nav2 servers (no map server, no AMCL)
  launch/goal_navigation.launch.py  Webots + driver + RViz + goal_navigator (no Nav2)
  agrobot_webots/goal_navigator.py  goes to the RViz goal, avoids obstacles in front
  agrobot_webots/waypoint_mission.py  sends a waypoint list to Nav2, logs CSV
  config/waypoints/        waypoint lists: the three study cases
  config/nav2_params.yaml  Nav2 parameters (map-free, odom frame)
  config/ekf.yaml          robot_localization EKF (wheels + IMU), used with ekf:=true
  config/slam_toolbox.yaml slam_toolbox (lidar SLAM), used with slam:=true
  config/ekf_map.yaml      map-frame EKF (wheels + IMU + slam_toolbox pose), used with slam:=true
  rviz/nav.rviz            RViz config with the "Nav2 Goal" tool
  config/rtabmap.yaml      RTAB-Map (RGB-D camera + lidar VSLAM), used with vslam:=true
  rviz/nav_slam.rviz       same, Fixed Frame map, with /map and the covariance ellipses (slam:=true)
  rviz/nav_vslam.rviz      same, Fixed Frame map, with the RTAB-Map grid and path (vslam:=true)
  rviz/goal.rviz           RViz config for goal_navigator ("2D Goal Pose" tool)
agrobot_gazebo/          Gazebo Harmonic worlds (.sdf) and launch files (ament_cmake)
agrobot_control/         Gazebo-era nodes: obstacle_avoidance.py, stability_monitor.py
vineyard_gazebo_world/   Third-party vineyard worlds for Gazebo (git-ignored)
utils/commands.md        Command notes
claude/                  Notes for/from Claude: instructions.md (plan + status), manual.md
```

## Setup

### Dependencies

```bash
sudo apt install ros-jazzy-webots-ros2 ros-jazzy-navigation2 ros-jazzy-rviz2 \
                 ros-jazzy-robot-state-publisher ros-jazzy-robot-localization \
                 ros-jazzy-slam-toolbox ros-jazzy-rtabmap-ros ros-jazzy-ros-gz
```

Webots runs on Windows. `webots_ros2_driver` finds it through `WEBOTS_HOME`, so that
variable must point to the Windows install (`/mnt/c/Program Files/Webots`). The
`$env:IPAddress` line in [utils/commands.md](utils/commands.md) is the PowerShell side of
that WSL ↔ Windows connection.

### Build

```bash
cd ~/projects/ros2_gazebo_mestrado
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install            # or: --executor sequential on low-RAM machines
source install/setup.bash
```

Rebuild `agrobot_webots` after editing anything in it. The launch files read the
**installed** copies of the world, PROTO, params and RViz config from
`install/agrobot_webots/share/...`.

## Running (Webots + Nav2)

```bash
ros2 launch agrobot_webots simulation.launch.py
```

This starts Webots with `obstacle_arena.wbt` (or the world given in `world`), the Agrobot
driver plugin, `robot_state_publisher`, the Nav2 servers and RViz. Closing Webots shuts the
whole launch down.

Launch arguments:

| Argument | Default | Effect |
|---|---|---|
| `world` | `obstacle_arena.wbt` | World file in `agrobot_webots/worlds/`. `plantation_field.wbt` is the crop field ([below](#plantation-field-world)) and `olive_grove.wbt` the olive grove ([below](#olive-grove-world)). `goal_navigation.launch.py` takes the same argument |
| `nav` | `true` | Start the Nav2 servers (`navigation.launch.py`) |
| `rviz` | `true` | Start RViz with `rviz/nav.rviz` (`rviz/nav_slam.rviz` with `slam:=true`, `rviz/nav_vslam.rviz` with `vslam:=true`) |
| `ekf` | `false` | `true`: the `robot_localization` EKF fuses `/wheel/odom` + `/imu/data` and publishes `/odom` and TF `odom → base_link` (the driver stops publishing them). `goal_navigation.launch.py` takes the same argument |
| `camera` | `true` | `false`: the driver doesn't enable the VSLAM RGB-D camera, so Webots doesn't render it and the `/vslam_camera/...` topics don't exist |
| `data_camera` | `false` | `true`: the driver enables the RGB data acquisition camera and publishes `/data_camera/image_raw` and `/data_camera/camera_info` (4.9 MB per image, about one per second). It can be switched off and on while running with `ros2 service call /data_camera/enable std_srvs/srv/SetBool "{data: false}"`. With `false` (default) Webots doesn't render it and neither the topics nor the service exist |
| `data_record` | `true` | With `data_camera:=true`, the `data_recorder` node saves the acquisition camera images as PNG, with the estimated and true camera pose of each one, in `dados/fotos/<world>_<mode>_<date>/` in the project (ignored by git) (`fotos.csv`, `camera_info.yaml`). `false` only shows the images ([details](claude/manual.md#6-câmera-rgb-de-aquisição-de-dados-data_cameratrue)) |
| `data_dir` | `dados/fotos/` in the project | Folder for the saved photos |
| `data_min_distance` | `0.3` | Minimum distance (m) the camera must move between two saved photos |
| `slam` | `false` | `true`: `slam_toolbox` builds `/map` from `/scan` and publishes TF `map → odom` and `/pose`; `ekf_map_node` fuses `/wheel/odom` + `/imu/data` + `/pose` into `/odometry/map`. RViz shows the map and the `/odometry/map` covariance ellipse (magnified 20×); an `/odom` ellipse for comparison is in the Displays panel, off by default and meaningful only with `ekf:=true`. Nav2 plans in `map` (bt_navigator, global costmap, behaviors); the controller and the local costmap stay in `odom`. Works with `ekf:=true` or `false` |
| `vslam` | `false` | `true`: RTAB-Map maps with the RGB-D camera and the lidar and publishes TF `map → odom`, `/rtabmap/map` and `/rtabmap/mapPath` ([VSLAM](#vslam-rtab-map)). Needs `ekf:=true` and `camera:=true`. With `slam:=true` too, slam_toolbox keeps running without TF, for comparison. Nav2 plans in `map`, as with `slam:=true` |
| `vslam_localization` | `false` | With `vslam:=true`: localize on a saved RTAB-Map database instead of mapping (the database isn't changed) |
| `vslam_db` | `dados/vslam/<world>.db` in the project | RTAB-Map database. Mapping erases it at start |
| `slip_check` | `true` | With `ekf:=true` and `camera:=true`: wheel-slip check. `rgbd_odometry` (RTAB-Map) measures the robot's speed from the RGB-D camera (`/vo/odom`), and `wheel_slip_monitor` compares it with the wheels. When they disagree (robot stuck with the wheels spinning, or dragged with them stopped), the EKF gets the camera's speed instead of the wheels' (`/wheel/odom_checked`). The CSV mode becomes `ekf-vo`. `false`: the EKF reads `/wheel/odom` directly, as before |
| `rtabmap_viz` | `false` | With `vslam:=true`: also open the `rtabmap_viz` window (loop closures, features) |
| `waypoints` | (empty) | Name of a waypoint list in `config/waypoints/` (without `.yaml`), a path to a `.yaml`, or `rviz` to click the waypoints in RViz. Starts `waypoint_mission`, which drives the robot through the list once Nav2 is up ([below](#waypoint-missions)) |

Examples:

```bash
ros2 launch agrobot_webots simulation.launch.py ekf:=true                 # Nav2 on the EKF odometry
ros2 launch agrobot_webots simulation.launch.py ekf:=true slam:=true      # + lidar SLAM and the map-frame ellipse
ros2 launch agrobot_webots simulation.launch.py nav:=false rviz:=false   # robot only
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt slam:=true  # crop field
ros2 launch agrobot_webots navigation.launch.py                          # Nav2 alone, in another terminal
```

### Sending a goal

- **RViz:** use the **Nav2 Goal** tool in the toolbar and click-drag on the grid. The fixed
  frame is `odom`.
- **CLI:**
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: odom}, pose: {position: {x: 2.6, y: 0.6}, orientation: {w: 1.0}}}}"
  ```

Goal coordinates are in `odom`. **`odom` = where the robot spawned** (world position
`(-1.0, -0.3)`, facing +x), not the center of the arena. The walls are at world ±2.5 m, so
in `odom` the free space runs roughly from x = -1.5 to 3.5 and y = -2.2 to 2.8, minus the
robot's half-width (~0.56 m). The goal must leave room for the whole 1.2 × 1.12 m footprint:
the old example `(1.5, 1.0)` puts the robot on top of the crate at world `(0.19, 0.37)` and
Nav2 aborts it.

The global costmap is a 24 × 24 m window centered on the robot, so Nav2 refuses goals more
than about 12 m away.

### Plantation field world

```bash
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt            # Nav2
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true slam:=true
ros2 launch agrobot_webots goal_navigation.launch.py world:=plantation_field.wbt       # no Nav2
```

A crop field on uneven ground (x along the rows, world frame):
- 18 × 16 m `ElevationGrid` of soil: wide gentle hills (20–40 m wavelength, up to ~5 cm),
  fine roughness, a 5 cm raised bed under each row, shallow wheel ruts and a few holes.
  Wheel–soil friction is 0.7 (Webots default 1).
- 6 rows of `CropPlant` (corn-like, 0.9–1.6 m tall), 9 m long (x = -4.5 to 4.5), 1.8 m
  apart (y = ±0.9, ±2.7, ±4.5), with gaps. The 5 lanes between them are at y = 0, ±1.8, ±3.6.
- In the lanes, clods, small stones and one branch each, all under the lidar plane: the
  robot drives over them. Six bigger rocks (15–23 cm above the ground, the lidar sees them):
  four in the headlands and two in the side margins, at world `(1.5, -5.4)` and `(-2.5, 5.5)`.
- Headlands 2.5 m long at both ends, a fence of wooden posts at x = ±7.6, y = ±6.6.
- The robot starts at world `(-6, 0)` facing +x, at the head of the middle lane. So in
  `odom` the lanes are at y = 0, ±1.8, ±3.6, the rows run from x = 1.5 to 10.5, and the
  far headland is around x = 11.5–12.5. Example: `(10.5, 0)` drives down the middle lane,
  `(11.8, 1.8, yaw π)` turns into the next lane, `(0.5, 1.8, yaw π)` comes back.

The world is generated. To change it, edit the parameters in
[generate_plantation_field.py](agrobot_webots/scripts/generate_plantation_field.py) and run
`python3 agrobot_webots/scripts/generate_plantation_field.py` (`--roughness 2` for rougher
ground, `--seed N` for another layout), then rebuild `agrobot_webots`.

**The lidar and the uneven ground.** `front_lidar` is a single horizontal plane about 13 cm
above the ground. When the robot pitches over a clod or a stone, the plane can still hit the
soil ahead, and Nav2 marks it as an obstacle in the lane, so expect some "collision ahead"
stops. In the cloud tests, 4 runs of a 5-goal route through the lanes reached all 20 goals
(with the earlier 8 cm lidar it was 18 of 23). If a goal aborts, resend it, and if it keeps
failing clear the costmaps first:

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

`goal_navigator` (below) does not get into the lanes: just before the first plants it
reports an obstacle in the strip it needs free in front of the robot, and it gives up after
40 s. It did the same with the 8 cm lidar; the cause wasn't investigated. Measurements are in
[claude/manual.md](claude/manual.md#3-o-robô-no-webots).

### Olive grove world

```bash
ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true vslam:=true
ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true vslam:=true \
    waypoints:=caso4_olival_ida_volta
```

A traditional olive grove modeled on one at Vilarinho dos Galegos (Mogadouro), on the Douro
Internacional plateau (x east along the rows, y north and uphill, world frame):
- Red tilled soil that rises to the north: 4–5% across the grove (about 2.4 m from the
  first to the last row). Over the grove, a 100 × 84 m `ElevationGrid` (0.5 m cells) makes
  the ground uneven: wide gentle waves, noise at three scales (12, 4 and 1.5 m) and
  scattered mounds and hollows (2–5 cm high, 1.2–2.5 m wide). Around it, a coarser 400 m
  grid follows the same slope. Wheel–soil friction is 0.8.
- 65 `OliveTree`s on a 7 × 7 m grid (7 rows of 10 places, x = -31.5 to 31.5,
  y = -21 to 21), with a few missing and young trees; 2.3–3.9 m tall, trunks 1–1.25 m.
  Only the trunk collides, and the lidar sees only the trunks: the tree lines are open, with few
  returns.
- Obstacles the lidar sees: 10 piles of pruned branches (1.6–2.6 m long, 0.45–0.75 m tall)
  and 2 stone heaps in the lanes, 2 stone heaps in the headlands, and 12 rock blocks (15–23 cm
  tall, half in the lanes, half between trees in the tree lines). They stay at least 2.5 m
  from the case 4 and case 5 waypoints, so Nav2 has to drive around the ones in the lanes.
- Things the robot drives over: fallen branches in the lanes (4–7 cm) and flat stone chips
  all over the grove (2–4 cm); small stones only in the tree lines.
- Around it: a vineyard to the north (4 rows with posts, y = 28.5 to 36), a dirt road to the
  south (y = -30.5 to -27) and an irrigation tank with a pump house to the east (x = 45).
- The robot starts at world `(-36, -3.5)` facing +x, in the west headland. So in `odom`/`map`
  the lanes between rows are at y = -14, -7, 0, 7, 14 and 21, the trees run from x = 4.5 to
  67.5, and the east headland is at x ≈ 68–74. The gaps across the rows (to drive up or down
  the slope) are at x = 8, 15, 22, 29, ...

The world is generated. To change it, edit the parameters in
[generate_olive_grove.py](agrobot_webots/scripts/generate_olive_grove.py) and run
`python3 agrobot_webots/scripts/generate_olive_grove.py` (`--seed N` for another layout,
`--slope 0.03` for a gentler hill, `--slope 0` for flat ground, `--noise 0.5` or `--noise 0`
for less or no unevenness), then rebuild `agrobot_webots`.

The bumps are wide and shallow on purpose. The lidar is a single plane about 13 cm above the
ground, and Nav2 marks obstacles up to 5 m away. In tests, bumps a few meters wide and more
than 2–3 cm high tilted the robot enough for that plane to hit the ground ahead, and Nav2
saw obstacles that weren't there (over 20 times more ground hits than on flat ground). With the
current values the hits are about the same as with `--noise 0`.

**Results in the cloud** (Webots R2025a headless, `ekf:=true vslam:=true slip_check:=false`,
4–5% slope, uneven ground and the obstacles above): both cases reached every waypoint, with
no "collision ahead" stops, no recovery spins and no phantom obstacles from the ground
(case 4: 15/15 in 450 s; case 5: 12/12 in 349 s). The robot slides downhill when it turns or
drives over a branch on the side slope, and the wheels and the IMU can't see that, so the
pose error grows to 1.6 m (case 4, all of it on the way back, where it drove around
obstacles) and 1.4 m (case 5). With the old 2–3% slope it was 0.6 m and 1.4 m. RTAB-Map
didn't correct it: on the way back the robot sees the trees from other angles and closes no
loop. `--slope 0.03` brings back the gentler slope and `--slope 0` gives a flat grove.

**Nav2 in open ground.** Webots returns +inf for lidar rays that hit nothing, and in the
grove most rays do. The costmaps now clear along those rays too (`inf_is_valid: true` in
`nav2_params.yaml`). Before that, only the slices toward trunks were known free, the planned
path zigzagged between them, and the skid-steer slip in all those turns put the EKF pose
about 1 m off after 50 m.

### Waypoint missions

```bash
ros2 launch agrobot_webots simulation.launch.py ekf:=true waypoints:=caso1_arena_circuito
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true slam:=true \
    waypoints:=caso2_plantacao_ida_volta
```

`waypoint_mission` reads the list, waits for Nav2 and for the TF from the global frame to
`base_link`, and sends the whole list to the Nav2 `waypoint_follower` (`FollowWaypoints`).
The follower drives to one waypoint at a time, stops there for 3 s (`WaitAtWaypoint`,
`waypoint_pause_duration` in `nav2_params.yaml`: the place for data acquisition) and, if a
waypoint fails, skips it and goes on (`stop_on_failure: false`).

A waypoint list ([config/waypoints/](agrobot_webots/config/waypoints)):

```yaml
world: plantation_field.wbt      # optional: the node refuses to run in another world
waypoints:
  - {name: meio_y0, x: 6.0, y: 0.0}
  - {name: cabeceira_leste, x: 11.3, y: 0.0, yaw: 1.5708}
```

- Coordinates are in the frame where the robot started: `odom` starts there, and so does
  the `map` of slam_toolbox and of RTAB-Map. The same list works with and without SLAM; the
  launch picks the frame (`map` with `slam:=true` or `vslam:=true`, otherwise `odom`).
- Without `yaw`, the robot arrives facing the next waypoint (the last one, facing the way it
  came), so it doesn't turn in place more than it needs to.
- Each waypoint must be less than ~12 m from the previous one (the global costmap window).

In RViz, the **Waypoints** display shows the list: orange pending, blue current, green
reached, red missed. At the end, the node writes two CSV files to `dados/missions/` in the project (ignored by git),
named `<list>_<mode>_<date>_*.csv`:
- `*_waypoints.csv`: one row per waypoint, with the status, the time, the estimated pose
  (TF), the true pose (`/ground_truth/odom`), the true error to the target and the
  localization error (estimated vs. true);
- `*_trajetoria.csv`: estimated and true pose at 5 Hz, for ATE/RPE.

The true pose is brought to the start frame (the robot's pose when the node starts), so it
compares directly with `odom`/`map`.

To compare runs, [summarize_missions.py](agrobot_webots/scripts/summarize_missions.py)
prints one Markdown table row per run: waypoints reached, time, true distance, true error at
the last waypoint, ATE (RMS of the localization error along the trajectory), and the largest
position and heading errors:

```bash
python3 agrobot_webots/scripts/summarize_missions.py   # reads dados/missions/; or pass folders/CSVs
```

With several runs of each case, [compare_methods.py](agrobot_webots/scripts/compare_methods.py)
groups them by case and mode: waypoints reached (as Nav2 reports them), waypoints where the
robot really was within 0.5 m of the target, mean ± standard deviation of the ATE, the largest
errors, and a list of the missed waypoints with the localization error at that moment:

```bash
python3 agrobot_webots/scripts/compare_methods.py   # reads dados/missions/; or pass folders/CSVs
```

**Waypoints clicked in RViz** (`waypoints:=rviz`):

```bash
ros2 launch agrobot_webots simulation.launch.py ekf:=true waypoints:=rviz
```

Pick the **Publish Point** tool in the RViz toolbar and click the map: each click adds a
waypoint (`wp1`, `wp2`, ...) to the end of the list, and the Waypoints display shows it. Then,
in another terminal:

```bash
ros2 service call /waypoint_mission/start std_srvs/srv/Trigger   # drive the list
ros2 service call /waypoint_mission/undo std_srvs/srv/Trigger    # remove the last waypoint
ros2 service call /waypoint_mission/clear std_srvs/srv/Trigger   # remove all of them
ros2 service call /waypoint_mission/save std_srvs/srv/Trigger    # write the list to a YAML file
```

- Clicks are converted to the mission frame (`odom`, or `map` with `slam:=true` or
  `vslam:=true`) and ignored while a mission runs. The Publish Point tool is in all three
  RViz configs (`nav.rviz`, `nav_slam.rviz`, `nav_vslam.rviz`).
- After a mission ends, `start` drives the same list again, and new clicks add to it.
- `save` writes `dados/missions/rviz_<date>.yaml` in the format above, with the
  world, so the list can be run again with `waypoints:=<that file>`.
- The CSV files are named `rviz_<mode>_<date>_*.csv`.

**Study cases** for comparing EKF, SLAM and VSLAM (each starts and ends at the start pose):

| List | World | Route | Length |
|---|---|---|---|
| `caso1_arena_circuito` | `obstacle_arena.wbt` | south of the center crate, east, north, east again, back to the start | ~10 m |
| `caso2_plantacao_ida_volta` | `plantation_field.wbt` | middle lane to the east headland, back along lane y = 1.8, to the start | ~26 m |
| `caso3_plantacao_serpentina` | `plantation_field.wbt` | lanes y = 0 (east), 1.8 (west), −1.8 (east), and 0 again (west) to the start | ~51 m |
| `caso4_olival_ida_volta` | `olive_grove.wbt` | lane y = 0 to the east headland, back along lane y = 7, to the start | ~160 m |
| `caso5_olival_encosta` | `olive_grove.wbt` | across the rows up the slope at x = 8, along the top lane, down at x = 29, back along lane y = -7 | ~115 m |

Run each case with `ekf:=true` (EKF only, Nav2 in `odom`), `ekf:=true slam:=true` (lidar
SLAM, Nav2 in `map`), `ekf:=true vslam:=true` (RTAB-Map mapping, Nav2 in `map`) and, after a
mapping run, `ekf:=true vslam:=true vslam_localization:=true` (RTAB-Map localizing on the
saved map). The mode goes into the CSV names (`ekf`, `ekf+slam`, `ekf+vslam`,
`ekf+vslam-loc`). The camera is on by default (`camera:=true`) in every mode, so the CPU load
is the same; `camera:=false` lightens the runs without VSLAM. The plantation lists have a
waypoint in the middle of each lane (x = 6): without it, Nav2 may take a different lane to
reach the far headland.

Comparison in the cloud (2026-10-29, three runs of each case and mode; tables, failures and
analysis in [claude/manual.md](claude/manual.md)):
- Arena (case 1): all three methods reached every waypoint. Lidar SLAM localized best (ATE
  0.03 m against 0.08 m for the EKF and for RTAB-Map).
- Case 2: RTAB-Map reached every waypoint in the three mapping runs (ATE 0.15 m) and 14/15
  localizing on the saved map (ATE 0.13 m). The EKF got 13/15 (ATE 0.29 m), lidar SLAM 10/15
  (ATE 2.1 m, heading up to 51° off).
- Case 3: the EKF got 30/33 waypoints (ATE 0.47 m), RTAB-Map 28/33 (ATE 1.05 m), lidar SLAM
  12/22 in two runs plus one run that didn't finish in 40 min.
- In the plantation, most failures came from the east headland, for every method: Nav2 can't
  plan the turn into the next lane, or takes the lane after it, or the robot gets stuck against
  a plant or the stone at (11.8, −1.5) with its wheels spinning. The spinning wheels move the
  odometry 1.5–1.7 m while the robot stays put; RTAB-Map, with global loop closure off, does
  not catch an error that large, and the robot then drives the wrong lane believing it is in
  the right one.

In the simulation the IMU yaw has no noise or drift, which favors the EKF; add sensor noise
before the final study runs.

### VSLAM (RTAB-Map)

```bash
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true vslam_localization:=true
ros2 run tf2_ros tf2_monitor map odom     # map → odom must come from one node only
```

- **Mapping** (`vslam:=true`). RTAB-Map takes the RGB-D camera and the EKF odometry
  (TF `odom → base_link`), adds a node about every second while the robot moves and
  closes loops only near the EKF estimate (proximity detection): the camera image against
  nodes up to 1 m and 45° away, using the odometry as the guess, and the lidar scan against
  the scans of nearby nodes, each checked by ICP. Appearance-based (global) loop closure is
  off (`Rtabmap/LoopThr` 1.01) because the plantation lanes look the same: it is what made
  the pose jump between lanes. It publishes TF
  `map → odom` at 20 Hz (identity until the first frame), the lidar occupancy grid
  `/rtabmap/map` and the graph path `/rtabmap/mapPath`, both shown by `rviz/nav_vslam.rviz`.
  The grid is assembled only while someone subscribes to `/rtabmap/map`, so keep RViz open
  while mapping. Drive around with Nav2 goals, a waypoint mission or teleop; Nav2 plans in `map`.
- **Database.** `dados/vslam/<world>.db` in the project (for example `plantation_field.db`), or
  the path given in `vslam_db`. Mapping **erases it when it starts**, so copy the file to
  keep a map. Inspect it with `rtabmap-info <file>`.
- **Localization** (`vslam_localization:=true`). RTAB-Map loads the saved map (a copy in
  `/tmp`, the file itself isn't changed) and adds nothing to it. It starts with the robot on
  the map's first node and corrects `map → odom` (still published at 20 Hz) when it
  recognizes a place near the EKF estimate (proximity detection, as in mapping); the first
  correction waits for a second recognition. The robot has to
  start where it started when mapping (the worlds fix the spawn pose).
- **With `slam:=true` too.** slam_toolbox keeps running for comparison, without TF: its map
  goes to `/slam_toolbox/map` in frame `slam_map`, which isn't in the TF tree (RViz can't
  show it). Save it with `ros2 run nav2_map_server map_saver_cli -t /slam_toolbox/map`.
  `ekf_map_node` and the `/odometry/map` ellipse don't run with `vslam:=true`.
- `rtabmap_viz:=true` opens the RTAB-Map window (loop closures, features).
- **The plantation soil texture repeats every 2 m**, and the camera sees mostly soil: in
  the first test, 8 loop closures placed the robot exactly 2 m away from where it was. The
  lidar check (ICP) and robust graph optimization dealt with most of them, but the lanes also
  look the same to the lidar (one 1.8 m closure from lane y = 0 to y = -1.8 passed the ICP in
  the serpentine case), so global loop closure is now off (see Mapping above). In the cloud test, 2 laps around the
  y = 0 and y = 1.8 lanes gave a `map` pose error of 0.11 m RMS (0.23 m max, 0.08 m at the
  end), against 0.28 m RMS (0.46 m max, 0.45 m at the end) for the EKF odometry alone.
  Details in [claude/manual.md](claude/manual.md#vslam-configrtabmapyaml-vslamtrue).
  The arena has plain walls and floor and few visual features.

### Goal without Nav2 (`goal_navigator`)

```bash
ros2 launch agrobot_webots goal_navigation.launch.py     # rviz:=false to skip RViz
```

Same Webots world and driver, but no Nav2 and no costmaps. In RViz (Fixed Frame `odom`),
pick **2D Goal Pose** and click-drag on the grid. The `goal_navigator` node drives
towards the goal using only `/odom`. The lidar is only used to check whether the strip in
front of the robot (its width plus a margin) is free. If the way to the goal is blocked, it
takes the free direction closest to the goal and heads for the goal again once that way
opens. It keeps the lidar points it saw near the robot for a while, because the front lidar
can't see the sides of the body, and checks them before every turn so the body doesn't
hit anything. It stops with a log message when it can't get closer to the goal for 40 s,
or when it is boxed in and can't turn or maneuver for 5 s ("Robô cercado").
Details and parameters: [claude/manual.md](claude/manual.md).

### Manual driving

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard        # with nav:=false
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"
```

When Nav2 is running, `velocity_smoother` also publishes on `/cmd_vel`. Use `nav:=false`
for manual driving.

### Useful checks

```bash
ros2 topic hz /scan                     # ~10 Hz
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames          # odom -> base_link -> wheels, front_lidar_link, ...
ros2 run tf2_ros tf2_monitor map odom   # with slam:=true or vslam:=true: one publisher, ~20 Hz
ros2 lifecycle get /bt_navigator        # should be "active [3]"
```

## Webots architecture

```
                 Webots (Windows)
                      │  extern controller
            webots_ros2_driver  ──loads plugin──►  AgrobotDriver (agrobot_driver.py)
                                                     │ subscribes /cmd_vel
                                                     │ publishes  /odom, TF odom→base_link (ekf:=false),
                                                     │            /wheel/odom, /imu/data, /clock,
                                                     │            /scan, /joint_states,
                                                     │            /vslam_camera/... (camera:=true)
ekf_filter_node (ekf:=true) ◄── /wheel/odom_checked (or /wheel/odom with slip_check:=false), /imu/data
                                ──► /odom, TF odom→base_link
rgbd_odometry (slip_check)  ◄── /rtabmap/rgbd_image ──► /vo/odom (camera-only speed, no TF)
wheel_slip_monitor          ◄── /wheel/odom, /vo/odom ──► /wheel/odom_checked, /wheel/slip
slam_toolbox (slam:=true)   ◄── /scan, TF odom→base_link ──► /map, /pose, TF map→odom
ekf_map_node (slam:=true)   ◄── /wheel/odom, /imu/data, /pose ──► /odometry/map (no TF)
rgbd_sync (vslam or slip)   ◄── /vslam_camera/... ──► /rtabmap/rgbd_image
rtabmap (vslam:=true)       ◄── /rtabmap/rgbd_image, /scan, TF odom→base_link
                                ──► /rtabmap/map, /rtabmap/mapPath, TF map→odom
   (with vslam:=true, slam_toolbox publishes no TF and its map moves to /slam_toolbox/map,
    and ekf_map_node doesn't run)
robot_state_publisher ◄── /joint_states             ─┘
   (URDF static TF: base_link → wheels, lidars, imu, ...)

Nav2:  bt_navigator → planner_server (NavFn) → controller_server (Regulated Pure Pursuit)
       controller_server ─► /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel ─► AgrobotDriver
waypoint_mission (waypoints:=...) ─FollowWaypoints─► waypoint_follower ─NavigateToPose─► bt_navigator
```

### Topics and frames

| Topic | Type | Publisher | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | velocity_smoother / you | `linear.x`, `angular.z` only |
| `/odom` | `nav_msgs/Odometry` | AgrobotDriver, or the EKF with `ekf:=true` | driver: wheel distance + IMU yaw |
| `/wheel/odom` | `nav_msgs/Odometry` | AgrobotDriver | wheels only (yaw from the wheels too), EKF input |
| `/wheel/odom_checked` | `nav_msgs/Odometry` | wheel_slip_monitor (`slip_check:=true`) | `/wheel/odom`, with `vx`/`vy` and their variances replaced by the camera's while the wheels slip. EKF input |
| `/wheel/slip` | `std_msgs/Bool` | wheel_slip_monitor | `true` while the wheels slip, transient local |
| `/vo/odom` | `nav_msgs/Odometry` | rgbd_odometry (`slip_check:=true`) | visual odometry of the RGB-D camera; only the twist is used. Frame `vo_odom` (not in TF) |
| `/imu/data` | `sensor_msgs/Imu` | AgrobotDriver | frame `imu_link`: orientation, gyro, accelerometer, sensor-data QoS |
| `/clock` | `rosgraph_msgs/Clock` | AgrobotDriver | Webots sim time, every step |
| `/scan` | `sensor_msgs/LaserScan` | AgrobotDriver | frame `front_lidar_link`, 300 rays, 3.0 rad FOV, 0.1–8 m, sensor-data QoS |
| `/joint_states` | `sensor_msgs/JointState` | AgrobotDriver | 4 wheel positions |
| `/vslam_camera/color/image_raw` | `sensor_msgs/Image` | AgrobotDriver (`camera:=true`) | `bgra8`, 424 × 240, every 96 ms (~10.4 Hz), frame `vslam_camera_optical_frame`, reliable QoS |
| `/vslam_camera/color/camera_info` | `sensor_msgs/CameraInfo` | AgrobotDriver | pinhole, no distortion, fx = fy ≈ 223.5 px |
| `/vslam_camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | AgrobotDriver | `32FC1`, planar depth in meters, registered to the color image; `+inf` outside 0.5–6 m. Same stamp as the color image |
| `/vslam_camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | AgrobotDriver | same as the color one |
| `/data_camera/image_raw` | `sensor_msgs/Image` | AgrobotDriver (`data_camera:=true`) | `bgra8`, 1280 × 960, every 1024 ms of simulation time, frame `data_camera_optical_frame`, reliable QoS. Stamped with the simulation step the image was rendered in |
| `/data_camera/camera_info` | `sensor_msgs/CameraInfo` | AgrobotDriver (`data_camera:=true`) | pinhole, no distortion, fx = fy = 640 px, same stamp as the image |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox (`slam:=true`) | 0.05 m cells, transient local, every 5 s |
| `/pose` | `geometry_msgs/PoseWithCovarianceStamped` | slam_toolbox (`slam:=true`) | frame `map`, one per processed scan, scan-matching covariance |
| `/odometry/map` | `nav_msgs/Odometry` | ekf_map_node (`slam:=true`) | frame `map`: wheels + IMU + `/pose`; the ellipse shown in RViz |
| `/rtabmap/rgbd_image` | `rtabmap_msgs/RGBDImage` | rgbd_sync (`vslam:=true`) | color + depth + `camera_info` of one frame, exact sync |
| `/rtabmap/map` | `nav_msgs/OccupancyGrid` | rtabmap (`vslam:=true`) | from the lidar, 0.05 m cells, transient local; built only while someone subscribes |
| `/rtabmap/mapPath` | `nav_msgs/Path` | rtabmap (`vslam:=true`) | one pose per node of the optimized graph |
| `/rtabmap/info` | `rtabmap_msgs/Info` | rtabmap (`vslam:=true`) | per processed frame: loop closures, timing |
| `/rtabmap/localization_pose` | `geometry_msgs/PoseWithCovarianceStamped` | rtabmap (`vslam:=true`) | only in iterations with a loop closure or proximity link (a localization, with `vslam_localization:=true`), and only while subscribed |
| `/ground_truth/odom` | `nav_msgs/Odometry` | AgrobotDriver | true pose of `base_link` from the Webots Supervisor, frame `world`, child `base_link_gt`, no TF. For measuring only |
| `/waypoints/markers` | `visualization_msgs/MarkerArray` | waypoint_mission | waypoints, names and route, transient local |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz (Publish Point tool) | with `waypoints:=rviz`, each click adds a waypoint |
| `/tf` | | AgrobotDriver or EKF, robot_state_publisher, slam_toolbox or rtabmap | `map → odom` (slam_toolbox with `slam:=true`, rtabmap with `vslam:=true`) and `odom → base_link` are dynamic, the rest is static |

All messages are stamped with **Webots simulation time** (`robot.getTime()`), and the
nodes run with `use_sim_time: true`, except `goal_navigator`. Since 2026-10-07 the driver
plugin publishes `/clock` with that time on every step, so the ROS clock of those nodes
follows the simulation. Before that, nothing published `/clock` and their clock stayed at 0.

### Robot parameters

Set in the URDF `<webots><plugin>` block ([agrobot.urdf](agrobot_description/urdf/agrobot.urdf)):

| Property | Value | Meaning |
|---|---|---|
| `wheelRadius` | 0.22 m | |
| `wheelSeparation` | 0.96 m | geometric track width |
| `wheelSeparationScale` | 1.33 | effective track = 0.96 × 1.33, calibrated for skid-steer slip in turns |
| `publishOdom` | true | driver publishes `/odom` + TF; the launch sets it to false with `ekf:=true` |
| `vslamCameraPeriod` | 96 ms | VSLAM camera period, a multiple of the 32 ms step; the launch sets it to 0 (off) with `camera:=false` |
| `dataCameraPeriod` | 1024 ms | Data acquisition camera period, a multiple of the 32 ms step; the launch sets it to 0 (off) unless `data_camera:=true` |

Sensors in the PROTO ([agrobot.proto](agrobot_webots/protos/agrobot.proto)):
- The robot is a Supervisor (`supervisor TRUE`, since 2026-10-26), so the driver can read
  its true pose and publish `/ground_truth/odom`
- `InertialUnit` `imu`, `Gyro` `gyro` and `Accelerometer` `accelerometer` at `imu_link`
  (0.35 m above `base_link`), published together on `/imu/data`
- `Lidar` named `front_lidar` at (0.65, 0, -0.10) from `base_link`. `base_link` rests
  0.23–0.24 m above the ground, so the scan plane is about 13 cm above the floor (z = 0.18 m
  in the arena's world frame, whose floor top is at z = 0.05). It sees the boxes (0.20 m
  above the floor) and the crates (0.30 m). Until 2026-10-09 it was 5 cm lower; see
  [claude/manual.md](claude/manual.md#3-o-robô-no-webots)
- `VslamCamera` ([VslamCamera.proto](agrobot_webots/protos/VslamCamera.proto)): a `Camera`
  `vslam_color` and a `RangeFinder` `vslam_depth` at the same pose, with the same 424 × 240
  resolution and 87° horizontal FOV, so the depth is registered to the color image. It sits
  at (0.45, 0, 0.38) from `base_link` (~0.61 m above the ground), pitched 15° down, like an
  Intel RealSense D455 (depth 0.5–6 m). URDF frames: `vslam_camera_link` (x forward) and
  `vslam_camera_optical_frame` (z forward, x right, y down). The old `camera_link` is left
  as it was because the Gazebo package uses the same URDF. Only for VSLAM; the data
  acquisition camera is the `DataCamera` below
- `DataCamera` ([DataCamera.proto](agrobot_webots/protos/DataCamera.proto)): a color
  `Camera` named `data_camera` for crop monitoring, 1280 × 960 with a 90° horizontal FOV
  (73.7° vertical). It sits on a mast over the robot's center, at (0, 0, 0.66) from
  `base_link` (~0.90 m above the ground), looking left (+y) with no pitch. With the robot in
  the middle of an inter-row, the stalk line is 0.9 m away and the image covers 1.80 m along
  the row and 1.35 m of height (~1.4 mm per pixel). URDF frames: `data_camera_link` (x out
  of the lens) and `data_camera_optical_frame` (z out of the lens, x right, y down). It is
  not used for localization

### Nav2 configuration (summary)

[config/nav2_params.yaml](agrobot_webots/config/nav2_params.yaml):
- No map. `global_frame: odom`; with `slam:=true` or `vslam:=true` the launch rewrites it
  to `map` for the bt_navigator, the global costmap and the behavior server
  (`navigation.launch.py` argument `global_frame`). The local costmap and the controller
  stay in `odom`. Both costmaps are rolling windows (local 4×4 m, global 24×24 m, 5 cm
  cells). Goals must be inside the global window (up to ~12 m away).
- `waypoint_follower` with a 3 s stop at each waypoint and `stop_on_failure: false`.
- `default_server_timeout` of the bt_navigator: 1000 ms (Nav2 default 20 ms). On a loaded
  CPU (`slam:=true`, or the camera with `vslam:=true`) the planner sometimes took longer to
  accept the request, and the whole goal aborted.
- Layers: `obstacle_layer` (from `/scan`) + `inflation_layer` (radius 0.7 m).
- Footprint 1.2 × 1.12 m (chassis length × outer wheel width).
- Planner: NavFn. Controller: Regulated Pure Pursuit, 0.4 m/s, rotate-to-heading on.
- Goal tolerance: 0.15 m / 0.3 rad.
- Limits (velocity_smoother): 0.5 m/s forward, 0.3 m/s reverse, 1.0 rad/s.

## Running (Gazebo, earlier work)

```bash
ros2 launch agrobot_gazebo simulation.launch.py   # obstacle_arena_world.sdf + spawn + bridge
ros2 launch vineyard_gazebo_world small.launch.py # vineyard world (spawn the robot separately)
ros2 launch agrobot_description spawn_agrobot.launch.py
```

In Gazebo, the robot uses the `gz-sim-diff-drive-system` plugin and the tall `lidar_link`
`gpu_lidar` from the same URDF. The Webots-only parts of the URDF (`<webots>` block,
`front_lidar_link`) don't affect Gazebo.

## Known issues

- **Odometry drift.** Skid-steer wheels slip, and the encoders can't see it. With the
  driver odometry (2026-10-02) the error was about 0.3 m per goal and about 0.4 m after a
  round trip; with the EKF, 0.28 m RMS and 0.45 m at the end of 2 laps (~50 m) on the crop
  field (2026-10-21). The EKF (`ekf:=true`) doesn't remove this: the IMU fixes the heading,
  but nothing it fuses measures the distance traveled. `slam:=true` or `vslam:=true`
  corrects it in the `map` frame (`map → odom`; 0.11 m RMS with `vslam:=true` in the same
  test), and Nav2 then plans and takes goals in `map`.
- **Wheel slip when the robot gets stuck.** Against the stone at the east headland of
  `plantation_field.wbt`, the wheels can spin with the robot stopped, and the wheel odometry
  then runs away (1.5 m in 6 s). `slip_check:=true` (the default with `ekf:=true`) catches
  it with the camera's visual odometry: pushing against a wall for 17 s, the `/odom` error
  stayed at 0.3 m instead of 7 m. Slips smaller than ~0.1 m/s + 30% of the speed aren't
  caught (normal skid-steer slip in turns is about that), and if the camera loses tracking
  for more than 3 s during a slip, the wheels come back.
- **Lidar SLAM loses its heading in the plantation.** At the east headland of
  `plantation_field.wbt`, slam_toolbox's yaw went 10–23° off in every waypoint run with
  `slam:=true`. The next lane then looks tilted in `map`, its waypoints land on a plant row and
  Nav2 can't plan to them. The slam_toolbox settings weren't tuned for the plantation yet.
- **East headland of the plantation.** In about half of the plantation runs with the EKF or
  RTAB-Map, Nav2 fails at the east headland: "Failed to create plan with tolerance
  of: 0.300000" on the turn into the next lane, the robot taking lane y = 3.6 instead of
  y = 1.8, or the robot stuck against a plant or the stone near (11.8, −1.5). Stuck with the
  wheels spinning, the odometry drifts 1.5–1.7 m in a few seconds. Not fixed yet.
- **`odom` is relative to the spawn point.** Goals don't refer to fixed world positions.
  Moving the spawn pose in the `.wbt` moves the whole goal frame.
- **Orphan Nav2 processes.** If you kill `ros2 launch` with SIGKILL, leftover nodes (e.g.
  `lifecycle_manager`) deactivate the next run's nodes. Before relaunching, run:
  `pkill -9 -f nav2_`
- **World/PROTO version mismatch.** `obstacle_arena.wbt` is `R2023a`, the PROTO is
  `R2025a`. It loads in the installed Webots, but Webots may ask to convert the world.
- **`agrobot_control` won't run as is.** `obstacle_avoidance.py` and
  `stability_monitor.py` sit in the package root, not in the `agrobot_control/` Python
  module, so the `obstacle_avoidance` entry point can't import them. `stability_monitor`
  has no entry point. Fix: move both into `agrobot_control/agrobot_control/` and add the
  missing entry point.
- **`stability_simulation.launch.py`** uses the relative world path
  `agrobot_gazebo/stability_test_world.sdf`, so it only works when started from the repo
  root.
- **`[Errno 17] File exists: .../meshes/chassis.stl` on `colcon build`.** A workspace built
  with `--symlink-install` before the chassis mesh moved into `agrobot_webots/meshes/`
  (2026-10-06) keeps an old link in `install/` to `agrobot_description`'s mesh, and colcon
  won't replace it. Run once from the workspace root, then rebuild:
  `rm install/agrobot_webots/share/agrobot_webots/meshes/chassis.stl`. If the build still
  fails, or after switching branches or between builds with and without
  `--symlink-install`, clean the package: `rm -rf build/agrobot_webots install/agrobot_webots`.
- **Committed `__pycache__/` files.** Add `__pycache__/` to `.gitignore` and `git rm --cached` them.

## Next steps

1. Check `ekf:=true slam:=true`, `ekf:=true vslam:=true` and the waypoint missions in RViz
   on Windows/WSL (so far they only ran headless in the cloud), and measure how fast the
   simulation runs there with the camera and RTAB-Map. Then run the study cases with each
   mode. VSLAM plan: `planejamento/camera_rgbd.md` in the project files.
2. Move the Nav2 setup to the vineyard world (rows, longer distances).
3. Fix the `agrobot_control` package layout. Decide whether the reactive avoider is still
   needed alongside Nav2.
4. Fill in `package.xml` descriptions and licenses.

For what changed in the Webots + Nav2 work and why, see [claude/manual.md](claude/manual.md).
