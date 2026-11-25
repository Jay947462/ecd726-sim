# ECD 726 Car Nav2 Simulation

The simulation models the RACECAR/J FlatNose rover and runs an autonomous oval with cones. Use Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic.

## Build

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

To record metrics, start this in a separate terminal before the driver.

```bash
python3 -B "$REPO/oval_nav2/record_oval_metrics.py" \
  --laps 3 --output-prefix "$HOME/ecd726-oval-ws/oval-run"
```
