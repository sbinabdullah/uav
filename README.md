# Autonomous Search-and-Rescue Grid Explorer (Gazebo + PX4 + ROS 2)

A ROS 2 system that connects to PX4 SITL through the **Micro-XRCE-DDS** bridge,
flies an autonomous boustrophedon (grid) search pattern over a simulated
disaster zone in **Gazebo**, detects target objects with a downward camera
(ArUco markers + color anomalies), geotags them to WGS-84, and logs GPS
coordinates of every finding. A pymavlink sidecar cross-checks the MAVLink
telemetry channel against the DDS-derived findings.

![status](https://img.shields.io/badge/status-working-brightgreen)

## Verified results (SITL, 2026-08-05)

- Full autonomous mission: takeoff → 4-lane grid sweep → RTL → land/disarm
- **7/7 targets detected, median geotag error 0.64–0.99 m over 3 runs** (max 2.4 m)
- Artifacts per run: `findings.csv`, `findings.geojson`, `sightings.csv`,
  `mavlink_log.csv`, `demo.mp4` (split-screen camera+map video)

## Stack

Ubuntu 26.04 · ROS 2 **Lyrical** · Gazebo **Jetty** (ROS-vendored) ·
PX4-Autopilot main SITL · Micro-XRCE-DDS-Agent v3.0.1 (system FastDDS 3.6) ·
`px4_msgs` / `px4_ros_com` · OpenCV ArUco · pymavlink

## Repo layout

```
config/            mission.yaml (grid/AOI params), targets.yaml (ground truth), rviz/
sim/               worlds/sar_zone.sdf, models/ (aruco panels, x500_sar), airframes/
src/
  sar_mission/     grid_mission_node (C++ offboard control) + coverage planner
  sar_perception/  detection_node (ArUco + HSV color), geotag_node (geotagging)
  sar_viz/         telemetry_viz_node (paths, 2D mission map image)
  sar_msgs/        Detection.msg
  sar_bringup/     launch files + ros_gz_bridge config
scripts/           stack_up.sh / stack_down.sh (tmux orchestration),
                   install_sim.sh, gen_aruco.py, score_mission.py,
                   calibrate_cam.py, mavlink_monitor.py, record_demo.py
```

## Setup (from scratch)

```bash
# 1. ROS 2 Lyrical + Gazebo Jetty (vendored) + PX4 deps
sudo apt install ros-lyrical-ros-gz
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools

# 2. Micro-XRCE-DDS Agent v3 against ROS's FastDDS (required, see note)
#    + Micro-CDR v2.0.2 + Micro-XRCE-DDS-Client v3.0.1 (superbuild off)
cd Micro-XRCE-DDS-Agent/build && cmake .. -DUAGENT_SUPERBUILD=ON \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DCMAKE_PREFIX_PATH=/opt/ros/lyrical && make && sudo make install

# 3. PX4: enable XRCE client v3, build SITL
echo 'CONFIG_UXRCE_DDS_CLIENT_USE_DDS_V3=y' >> \
  PX4-Autopilot/boards/px4/sitl/default.px4board
make -C PX4-Autopilot px4_sitl_default

# 4. px4_msgs + px4_ros_com workspace (px4_ros_com needs the Lyrical patch)
# 5. this repo's workspace: colcon build
```

> **Why agent v3 + client v3:** PX4 docs still recommend agent v2.4.3, which
> pulls FastDDS 2.x — with ROS Lyrical's FastDDS 3.x the `/fmu/out/*` topics
> never appear in ROS. Both sides must run v3. See the tracked working notes
> for the full story (kept out of git).

## Run the demo

```bash
# one-time: install world/models/airframe into the PX4 tree
scripts/install_sim.sh

# every run:
scripts/stack_up.sh "sar_bringup demo.launch.py"   # agent + SITL + all nodes
python3 scripts/mavlink_monitor.py                 # MAVLink sidecar (optional)
python3 scripts/record_demo.py output/demo.mp4     # split-screen video (optional)
scripts/stack_down.sh

# after the flight:
python3 scripts/score_mission.py                   # accuracy report vs ground truth
```

With a display, add `rviz:=true` to the launch for the RViz view
(`config/rviz/sar.rviz`). On a desktop, record Gazebo + RViz with OBS instead
of `record_demo.py` (which exists for headless environments).
