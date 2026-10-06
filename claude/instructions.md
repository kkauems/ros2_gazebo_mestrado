# Nav2 "click a point, robot goes there" in agrobot_webots

## Context
`agrobot_webots` (ROS 2 Jazzy, Webots, skid-steer 4 wheels) currently only listens to `/cmd_vel`. It publishes no odometry, no TF, no sensors, and Nav2 isn't installed. Goal: use Nav2 to drive to a point picked in RViz. No mapping/SLAM/AMCL/map_server for now.

Design choice: run Nav2 **without a map**, with `odom` as the global frame (`global_frame: odom`) and rolling-window costmaps. Localization = wheel odometry (fused with IMU optional later). This is the minimal Nav2 setup that accepts a goal pose (RViz "Nav2 Goal" tool / `/goal_pose`).

## Steps

1. **Install** (user runs): `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb*`(not needed) → just `navigation2` + `nav2-bringup`. RViz is `ros-jazzy-rviz2`.

2. **Fix the driver plugin** `agrobot_webots/agrobot_webots/agrobot_driver.py`
   - Remove `rclpy.init()` and the private node; use the node webots_ros2_driver provides (`self.__node = webots_node`/`properties`-style `init(webots_node, properties)` pattern, create subscription on `self.__node`). Current code likely conflicts with the host's rclpy context.
   - Keep skid-steer kinematics (wheel radius 0.22, separation 0.96). Note: skid-steer slips when turning, so effective separation is larger; add a `wheel_separation_scale` parameter (start ~1.0, tune).
   - Subscribe `/cmd_vel` (Nav2 controller output; use remap if Nav2 outputs `cmd_vel_nav`).
   - Delete or clearly mark the unused duplicate `controllers/agrobot_driver/agrobot_driver.py`.

3. **Odometry (the main gap)** — in the same plugin
   - Read the 4 existing `PositionSensor`s (`<joint>_sensor`, `robot.getDevice`, `enable(timestep)`), integrate x, y, yaw from left/right wheel deltas, publish `nav_msgs/Odometry` on `/odom` and broadcast TF `odom -> base_link` (`tf2_ros.TransformBroadcaster`). Stamp with sim time.
   - Alternative if drift in yaw is bad: add an `InertialUnit` to the PROTO and use its yaw (step 5).

4. **TF tree / launch** `launch/simulation.launch.py`
   - Add `robot_state_publisher` (from the URDF file contents; static `base_link -> wheels/lidar/...`).
   - Add `base_footprint` is optional; keep `base_link` as `robot_base_frame`.
   - Add `ros2 launch nav2_bringup`-style nodes: include a new `launch/navigation.launch.py` that starts `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `smoother_server`, `velocity_smoother`, `lifecycle_manager` (via `nav2_bringup/navigation_launch.py` with `params_file` = ours, `use_sim_time:=true`; no `map_server`/`amcl` since we don't use localization launch).
   - Launch `rviz2` with a saved config (`rviz/nav.rviz`: fixed frame `odom`, RobotModel, TF, Odometry, Path, local costmap, Nav2 Goal tool).
   - Add a shutdown handler for Webots exit (`webots._supervisor`/`launch.actions.RegisterEventHandler(OnProcessExit)`).

5. **Obstacle sensing (so the local costmap sees crates/boxes)** — PROTO + URDF
   - Add a `Lidar` device to `protos/agrobot.proto` named e.g. `lidar`, mounted at the URDF `lidar_link` pose (0.20, 0, 0.28), and declare it in the URDF `<webots>` block: `<device reference="lidar" type="Lidar"><ros><topicName>/scan</topicName><frameName>lidar_link</frameName>...`. Publishes `/scan` with `frame_id lidar_link`.
   - Without this, Nav2 still works but ignores obstacles (empty costmap) and will drive through the crates. Do this as a second milestone if you want to first prove "goes to point" in an empty run.
   - Optional: `InertialUnit`/`Gyro` for `robot_localization` EKF (installed) later.

6. **Nav2 params** new file `agrobot_webots/config/nav2_params.yaml`, installed via `setup.py` data_files (add `config/*.yaml`, `rviz/*`, new launch).
   - `bt_navigator`: `global_frame: odom`, `robot_base_frame: base_link`, `odom_topic: /odom`, use `navigate_to_pose` default BT.
   - `controller_server`: Regulated Pure Pursuit (`nav2_regulated_pure_pursuit_controller`) — good fit for skid-steer, `max_vel_x ~0.5`, `max_vel_theta ~1.0`, `use_rotate_to_heading`, goal tolerances xy 0.15 / yaw 0.3.
   - `planner_server`: `NavfnPlanner` (`allow_unknown: true`) or SmacPlanner2D.
   - `local_costmap`: `global_frame: odom`, `rolling_window: true`, 4x4 m, resolution 0.05, `footprint` rectangle 1.2 x 0.8 (`[[0.6,0.4],[0.6,-0.4],[-0.6,-0.4],[-0.6,0.4]]`), plugins `obstacle_layer` (scan, `/scan`) + `inflation_layer`.
   - `global_costmap`: also `global_frame: odom`, `rolling_window: true`, `width/height ~20`, same layers (no static layer).
   - `use_sim_time: true` everywhere.

7. **Arena** (small, optional but recommended): `worlds/obstacle_arena.wbt` has no walls (5x5 floor) — add boundary walls or keep goals inside ±2 m so the robot doesn't drive off. Also the world is `R2023a` while the PROTO is `R2025a`; confirm the installed Webots version loads it.

## Critical files
- modify: `agrobot_webots/agrobot_webots/agrobot_driver.py`, `agrobot_webots/launch/simulation.launch.py`, `agrobot_webots/setup.py`, `agrobot_webots/package.xml` (add `nav2_bringup`, `nav_msgs`, `tf2_ros`, `robot_state_publisher`, `rviz2` exec_depends), `agrobot_webots/protos/agrobot.proto`, `agrobot_description/urdf/agrobot.urdf` (Webots `<device>` entries)
- new: `agrobot_webots/config/nav2_params.yaml`, `agrobot_webots/launch/navigation.launch.py`, `agrobot_webots/rviz/nav.rviz`

## Verification
Incremental, each step before moving on:
1. `colcon build --packages-select agrobot_webots agrobot_description && source install/setup.bash && ros2 launch agrobot_webots simulation.launch.py`
2. Teleop check: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"` → robot moves; `ros2 topic echo /odom` shows x increasing; `ros2 run tf2_tools view_frames` shows `odom -> base_link -> ...`; commanding a 1 m move/360° spin matches odom (tune separation scale).
3. `ros2 topic hz /scan` (once lidar added), visible in RViz.
4. `ros2 launch agrobot_webots navigation.launch.py` → `ros2 lifecycle get /bt_navigator` is `active`.
5. In RViz click **Nav2 Goal** at a spot (or `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: odom}, pose: {position: {x: 1.5, y: 1.0}, orientation: {w: 1.0}}}}"`): robot plans a path, steers around crates, stops within tolerance; `Goal succeeded` in log.

## Known risks
- Skid-steer odometry drift/slip on turns (tune separation; later fuse IMU with `robot_localization`).
- `odom` is a drifting frame: goals are relative to the start; fine for this scope, AMCL/SLAM replace it when mapping is added.

## Revisions after checking the code (2026-10-02)
1. **Step 2 is wrong — keep `rclpy.init()` + private node.** That *is* the official webots_ros2 Python-plugin pattern (`webots_node` exposes only `.robot`, no ROS node). Only the duplicate `controllers/agrobot_driver/` is removed.
2. **Lidar height (step 5):** `lidar_link` is at base_link z +0.28 → ~0.57 m above the floor; crates are 0.3 m and boxes 0.2 m tall, so a lidar there sees **nothing**. Add a separate low front lidar `front_lidar_link` at (0.65, 0, -0.15) (~0.14 m above floor, ahead of the wheels, 180° FOV so it doesn't hit them). `lidar_link` is left alone because the Gazebo package uses the same URDF.
3. **Nav2 launch (step 4):** Jazzy's `nav2_bringup/navigation_launch.py` also starts `collision_monitor`, `waypoint_follower` and `docking_server`; docking_server fails to configure without dock plugins, which aborts the lifecycle manager. Write our own `navigation.launch.py` that starts only controller/planner/smoother/behavior/bt_navigator/velocity_smoother + lifecycle_manager. cmd_vel chain: controller → `cmd_vel_nav` → velocity_smoother → `cmd_vel`.
4. **Footprint:** the wheels stick out past the chassis (y = ±0.48 ± 0.07, x = ±0.35 ± 0.22) → footprint ≈ 1.2 × 1.12 m, not 1.2 × 0.8.
5. **Start pose:** the robot spawns at (0,0), overlapping `plastic crate(4)` at (0.19, 0.37). Move the spawn to a free spot, (-1.0, -0.3).
6. **Odom stamps:** stamp with `robot.getTime()` (sim time), so we don't depend on the private node getting `/clock`.
7. Arena walls: add 4 low walls at ±2.5 m so the robot can't drive off and the lidar has something to see.
8. Install needed (no sudo in the agent): `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`.

## Status (2026-10-02) — done, verified end to end in Webots
- `/odom` + TF `odom->base_link` from the driver plugin. Wheel distance + `InertialUnit` yaw (matches Webots ground truth). `wheelSeparationScale` = 1.33.
- `/scan`, `/joint_states` are published **by the plugin**, stamped with `robot.getTime()`. webots_ros2_driver's own Ros2Lidar stamped every scan with t=0 (its node clock never advances), and Nav2's TF filter silently dropped all of them. The URDF `<device>` for the lidar is disabled for that reason.
- Costmaps: the lidar is 0.15 m below base_link (z<0 in odom), so both `obstacle_layer.min_obstacle_height` **and** `obstacle_layer.scan.min_obstacle_height` are -1.0. With only the source-level one, clearing works but nothing is marked. Inflation 0.7 (≥ inscribed radius 0.57).
- Nav2 goal tests: (2.2, 1.2) and back to (0, 0) → both SUCCEEDED, path goes around crate(4), no object moved.
- Known limit: skid-steer turns slip sideways in ways that encoders can't see → ~0.3 m true-position error per goal, ~0.4 m after a round trip. Next step: robot_localization (IMU accel) or GPS/AMCL.
- Gotcha when testing: killing `ros2 launch` with SIGKILL/pkill on the shell leaves orphan nodes (e.g. lifecycle_manager), which deactivate the next run's nodes. `pkill -9 -f nav2_` before relaunching.
