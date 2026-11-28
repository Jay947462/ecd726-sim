# ECD 726 FlatNose rover

This repository contains the RACECAR/J FlatNose simulator and its Jetson Orin
Nano integration for track learning and known-oval driving. The simulator
uses Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. On the Jetson, Jazzy runs
in the existing Docker workspace.

## Learn a physical track on the Jetson

The track-learning mode starts with a fresh map. It learns a single closed
corridor from LiDAR-visible boundaries and saves the map and route at the
end of the first lap. The first hardware profile caps mapping at 0.10 m/s
and replay at 0.15 m/s. The car stops after mapping so the operator can
review the route and explicitly re-arm for replay.

Follow the [Jetson track-learning guide](hardware_overlay/docs/TRACK_LEARNING.md)
for the Docker and driver workspace setup, the calibrated LiDAR and odometry
inputs, build commands, Foxglove, arming, and stop checks. On the Jetson,
run the wall-clock launch with the measured turn radius.

```bash
ros2 launch ecd726_description learn_track_hardware.launch.py \
  min_turn_radius:="${MEASURED_TURN_RADIUS_M:?Set the measured radius in metres}"
```

Start with USB-free ROS preview and a wheels-raised stop test. The physical
workflow requires a calibrated odometry publisher and measured steering,
footprint and stopping distance. The ROS stop releases motor current and the
car coasts. Use the independent physical propulsion cutoff during floor tests.

## Known oval on the Jetson

The Jetson runs the oval planner, Nav2 and the VESC drive guards onboard. Nav2
combines the physical map, odometry and LiDAR scan to generate drive commands.
Collision Monitor checks those commands before the autonomy adapter converts
them to ERPM and steering commands. The initial profile is forward-only, caps
speed at 0.15 m/s and starts disarmed. A stop or loss of a required input latches
the guarded stop. The ROS software stop releases current and the car coasts.

The physical VESC driver workspace and Foxglove dashboard are maintained in
[Kenou2005/ECD726-CAR](https://github.com/Kenou2005/ECD726-CAR). Use your existing
checkout on the Jetson. On a fresh Jetson, authenticate to that GitHub
repository and clone it as the host workspace.

```bash
git clone https://github.com/Kenou2005/ECD726-CAR.git "$HOME/ros2_ws"
```

From that workspace, add this branch and apply the guarded autonomy overlay.
These commands assume the host workspace is `$HOME/ros2_ws`.

```bash
cd "$HOME/ros2_ws"
git clone --branch feature/track-learning-robot --single-branch \
  https://github.com/Jay947462/ecd726-sim.git src/ecd726-sim
bash src/ecd726-sim/hardware_overlay/apply.sh "$PWD"
```

Follow the [Jetson and Foxglove bring-up guide](hardware_overlay/docs/OVAL_AUTONOMY.md)
for container setup, the USB-free manual preview, dashboard import, VESC and
LiDAR checks, map and calibration, and autonomous arming. It also covers
Foxglove monitoring during an onboard run and the SSH stop commands.

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

## Learn an unknown track in Gazebo

The learning mode starts online SLAM with live LiDAR and odometry. Closely
spaced LiDAR-visible boundaries must define a single closed corridor. On the
first lap the driver finds a forward path between the boundaries and checks
space for the rover's footprint. It checks the return to the start and saves
the learned map and a closed path. It may collect a short overlap past the
start gate to align the map before recording. Later laps use the learned path
as the reference for MPPI. SLAM keeps providing localization, and the driver
uses live scans to steer around obstacles.

Install SLAM Toolbox if needed, then rebuild the simulator as above.

```bash
sudo apt install ros-jazzy-slam-toolbox
```

End your current simulation before starting the learning session in the same
ROS domain. In a sourced terminal, start the wall-only circle. Use `track:=oval` for the
original oval with cones. Add `gui:=true` to see the run in Gazebo.

```bash
ros2 launch ecd726_description learn_track.launch.py track:=circle
```

In a second sourced terminal, set `REPO` as above and start the driver. Choose
a new output directory for each run. The lap count includes the mapping lap.

```bash
python3 -B "$REPO/track_learning/drive.py" \
  --laps 2 --output-dir "$HOME/learned-circle"
```

The defaults cap the mapping lap at 0.20 m/s and the later laps at 0.42 m/s.
Press Ctrl+C in the driver terminal to stop. Each successful run writes
`map.yaml`, `map.pgm`, `learned_route.json` and `run.json`. The saved route
uses the SLAM map frame. This launch is for Gazebo and uses simulated time.
