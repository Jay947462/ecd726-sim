# ECD 726 FlatNose rover

This repository contains the RACECAR/J FlatNose simulation and the physical oval
integration for a Jetson Orin Nano. The simulator uses Ubuntu 24.04, ROS 2 Jazzy
and Gazebo Harmonic. On the Jetson, Jazzy runs in the existing Docker workspace.

## Jetson Orin Nano and Foxglove

The Jetson runs the oval planner, Nav2 and the VESC drive guards onboard. Nav2
combines the physical map, odometry and LiDAR scan to generate drive commands.
Collision Monitor checks those commands before the autonomy adapter converts
them to ERPM and steering commands. The initial profile is forward-only, caps
speed at 0.15 m/s and starts disarmed. A stop or loss of a required input latches
the guarded stop. The ROS software stop releases current and the car coasts.

Start from your team's existing `ECD726-CAR` checkout on the Jetson. The overlay
uses that workspace's VESC driver, firmware preparation helper and Foxglove
dashboard. This example assumes the host checkout is `$HOME/ros2_ws`.

```bash
cd "$HOME/ros2_ws"
git clone --branch feature/jetson-oval-autonomy --single-branch \
  https://github.com/Jay947462/ecd726-sim.git src/ecd726-sim
bash src/ecd726-sim/hardware_overlay/apply.sh "$PWD"
```

Follow the [Jetson and Foxglove bring-up guide](hardware_overlay/docs/OVAL_AUTONOMY.md)
for container setup, the USB-free manual preview, dashboard import, VESC and
LiDAR checks, map and calibration, and autonomous arming. It also covers
Foxglove monitoring during an onboard run and the SSH stop commands.

The software stop, watchdogs and guarded flow were tested in ROS preview and
Gazebo. Physical stopping distance, calibrated steering and speed, localization,
and cone clearance must be checked on the car and course before autonomous laps.

## Build the simulator

Set `REPO` to your checkout path. This example uses the `Senior_Project` folder on WSL.

```bash
source /opt/ros/jazzy/setup.bash
REPO=/mnt/c/Senior_Project/ecd726-sim
mkdir -p "$HOME/ecd726-oval-ws"
cd "$HOME/ecd726-oval-ws"
colcon build --symlink-install --base-paths "$REPO" --packages-select ecd726_description
```

## Run the oval

In each terminal, source Jazzy and the built workspace, and set `REPO` to your checkout path.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/ecd726-oval-ws/install/setup.bash"
REPO=/mnt/c/Senior_Project/ecd726-sim
```

1. Start Gazebo and RViz. Press Play in Gazebo if it opens paused.

   ```bash
   ros2 launch ecd726_description display.launch.py use_sim_time:=true
   ```

2. Start Nav2 in a second terminal. Wait for `lifecycle_manager_navigation` to report that its managed nodes are active.

   ```bash
   ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true \
     map:="$REPO/oval_nav2/track.yaml" \
     params_file:="$REPO/oval_nav2/oval_nav2_stable_042.yaml"
   ```

3. Run the driver in a third terminal. The validated lap setup sends the custom oval path directly to MPPI.

   ```bash
   python3 -B "$REPO/oval_nav2/drive_stable_pass_v2.py" \
     --laps 3 --direct-reference \
     --replan-log "$HOME/ecd726-oval-ws/replans.jsonl"
   ```

   The driver prints `Requested lap count completed.` when it finishes. Restart Gazebo and Nav2 before the next attempt to reset the spawn pose and localization.
   Press Ctrl+C in the driver terminal to cancel the active Nav2 goal.

To record metrics, start this in a separate terminal before the driver.

```bash
python3 -B "$REPO/oval_nav2/record_oval_metrics.py" \
  --laps 3 --output-prefix "$HOME/ecd726-oval-ws/oval-run"
```
