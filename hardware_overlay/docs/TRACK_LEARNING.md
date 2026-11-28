# Learn a physical track on the Jetson

The rover builds a map from live LiDAR and odometry on the first lap, then stops
so you can inspect the map and learned route. After you re-arm it, Nav2 follows
the saved route with live obstacle checks. The hardware driver caps commanded
speed at 0.10 m/s for mapping and 0.15 m/s for replay. Bring the rover to a
marked start facing forward along the course. SLAM starts with a fresh map.

## Install on the Jetson

Use the existing JetPack installation and private ECD726-CAR checkout. The
examples use `$HOME/ros2_ws` on the Jetson host, with Git, Python 3 and `tmux`
available there. For a fresh checkout, first authenticate to GitHub with an
account that can access the hardware repository.

```bash
git clone https://github.com/Kenou2005/ECD726-CAR.git "$HOME/ros2_ws"
```

From the ECD726-CAR root, add the track-learning branch if this workspace has
no `src/ecd726-sim` checkout yet.

```bash
cd "$HOME/ros2_ws"
git clone --branch feature/track-learning-robot --single-branch \
  https://github.com/Jay947462/ecd726-sim.git src/ecd726-sim
```

If the oval autonomy checkout is already present, switch it to the new branch
after committing or saving any local changes you want to keep.

```bash
cd "$HOME/ros2_ws"
git -C src/ecd726-sim remote set-branches --add origin feature/track-learning-robot
git -C src/ecd726-sim fetch origin
git -C src/ecd726-sim switch feature/track-learning-robot
git -C src/ecd726-sim pull --ff-only
```

Apply the overlay and install the hardware dependencies from the same workspace.
Keep your calibrated local autonomy configuration with the hardware checkout.

```bash
bash src/ecd726-sim/hardware_overlay/apply.sh "$PWD"
python3 tools/fetch_dependencies.py
```

If needed, set up `vesc-firmware-tools` using `docs/VESC_TOOL_SETUP.md` in
ECD726-CAR. Use the container that mounts this checkout at `/work` and maps the
actual VESC. From the Jetson host, install the ROS packages inside it.

```bash
docker start vesc-firmware-tools
docker exec --user root vesc-firmware-tools sh -ec \
  'test -e /ros2_ws || ln -s /work /ros2_ws'
docker exec --user root vesc-firmware-tools bash -ec '
  apt-get update
  apt-get install -y python3-colcon-common-extensions python3-pytest \
    ros-jazzy-serial-driver ros-jazzy-xacro ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander ros-jazzy-slam-toolbox ros-jazzy-foxglove-bridge'
docker exec -it vesc-firmware-tools bash
```

Inside the container, build the workspace and run the guard's preview tests.
These tests exercise the ROS guard logic using preview outputs.

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-select \
  vesc_msgs vesc_driver car_drive ecd726_description \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
python3 -m pytest -q src/car_drive/test/test_autonomy_preview.py
```

For each autonomy terminal below, open a shell in the container and source the
same installation. The hardware helper defaults to ROS domain 77. Start the
sensor and odometry publishers in the same domain.

```bash
# On the Jetson host
docker exec -it -e ROS_DOMAIN_ID=77 vesc-firmware-tools bash

# Inside the container
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

## Check the rover and course

- Verify the firmware command timeout and the independent physical propulsion
  cutoff. With the wheels raised and clear, test the software stop, driver exit,
  lost commands and USB loss. Observe motor current and ERPM. On the track
  surface, measure how far the rover coasts at each intended speed. Keep a
  spotter with the cutoff ready during any floor test. Follow the motor hardware
  bring-up in `src/car_drive/README.md` in ECD726-CAR.
- Use a closed, level course with a single corridor, continuous walls or barriers
  at LiDAR height, and enough room for the full swept footprint at every turn.
  Keep the course clear for the first mapping and replay run. The default lap
  gate requires at least 8 m of travel before it can confirm a return.
- Calibrate the speed conversion, steering center and endpoints. Set up
  `src/car_drive/config/autonomy.local.yaml` from the `autonomy.hardware.yaml`
  template for a first-time install. Keep the autonomy speed limit at or below
  0.15 m/s and use the separately tested positive forward ERPM cap. Measure
  both turning radii under load and use the larger value, in metres, for
  `MEASURED_TURN_RADIUS_M`. The hardware launch and driver accept measured
  radii of at least 0.65 m. Confirm that the car's envelope with the wheels
  turned fits the configured footprint and padding before arming.
  [Hardware calibration](OVAL_AUTONOMY.md#prepare-the-physical-inputs)
- With the car disarmed, verify real `/scan` and calibrated `/odom` with current
  wall-clock stamps. This launch uses your existing sensor and odometry
  publishers. The odometry message needs frame `odom`, child frame
  `base_link` or `base_footprint`, and measured linear velocity. Verify
  `odom → base_link`, the measured TF from `base_link` to the scan's
  `header.frame_id`, and the transform to `base_footprint`. Check the scan
  overlay while stationary and during manual movement before autonomy.
  For the UST-10LX, configure the driver with the sensor's Ethernet address
  and the measured scan frame. [Hokuyo ROS 2 driver](https://github.com/Hokuyo-aut/urg_node2)

```bash
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_footprint base_link
```

Use `ros2 run tf2_ros tf2_echo base_link "$LIDAR_FRAME"` after setting
`LIDAR_FRAME` to the frame ID shown in the scan header. If your odometry uses
another topic, set `odom_topic` in `autonomy.local.yaml`, pass
`odom_topic:=/your_odom` to the launch, and pass `--odom-topic /your_odom` to
the driver. The TF frame for local odometry is `odom`.

## Start SLAM and the guard session

End the manual controller and the known-oval launch and driver before starting
track learning. From the ECD726-CAR root on the Jetson host, set
`CHECKED_FORWARD_ERPM` to the measured cap and start the hardware guards.
The steering, drive and autonomy guards start disabled.

```bash
cd "$HOME/ros2_ws"
STEERING_AUTONOMY_CONFIG=src/car_drive/config/autonomy.local.yaml \
STEERING_MAX_ERPM="${CHECKED_FORWARD_ERPM:?Set the bench-tested forward cap}" \
  bash tools/start_steering.sh --hardware
```

For a preview of the guard and arming workflow, keep the propulsion hardware
disconnected and use `--preview` in place of `--hardware`. Check
`mode: preview` on `/drive/status`. Preview mode publishes mock motor commands
and requires a supplied sensor and odometry stream for the driver to arm.

With the car disarmed, place it at the start. In an autonomy ROS terminal, set
`MEASURED_TURN_RADIUS_M` and start the mapping and navigation nodes. The launch
starts SLAM Toolbox with wall time and a fresh map. Once active, check `/map`,
`map → odom`, and the map/scan alignment in Foxglove or RViz.

```bash
ros2 launch ecd726_description learn_track_hardware.launch.py \
  min_turn_radius:="${MEASURED_TURN_RADIUS_M:?Set the measured radius in metres}"
```

From another autonomy ROS terminal, check the transformation and navigation nodes.

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /controller_server
ros2 lifecycle get /collision_monitor
```

## Learn one lap and review the route

Set up the [stop terminal](#stop-and-reset) and keep it within reach. For remote
cancellation, start a dedicated `tmux` window on the Jetson host, then open a
sourced autonomy ROS shell inside it.

```bash
tmux new-session -s track-learner -n driver
```

In that ROS shell, set `MEASURED_TURN_RADIUS_M` to the same value used for the
launch. The output directory must be new. `--laps 2` counts the mapping lap and
one replay lap. `--laps 1` ends at the mapping stop.

```bash
RUN_DIR="/ros2_ws/track-runs/learn-$(date +%Y%m%d-%H%M%S)"
python3 -B /ros2_ws/src/ecd726-sim/track_learning/drive.py \
  --real-time \
  --min-turn-radius "${MEASURED_TURN_RADIUS_M:?Set the measured radius in metres}" \
  --output-dir "$RUN_DIR" --laps 2
```

The driver checks the sensors, SLAM, navigation and local path. It sends zero
commands through Collision Monitor while awaiting permission to move. When it
prints `Mapping lap ready`, use a second ROS terminal to enable the steering
and drive guards. Wait for both status topics to report `enabled: true`, then
enable the autonomy adapter. Verify each service response. Motion can start
when the final service succeeds.

```bash
ros2 service call /steering/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /drive/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: true}'
```

At the end of mapping the driver requests a latched stop, waits for standstill,
and saves `map.yaml`, `map.pgm` and `learned_route.json`. On the host, the files
are under `$HOME/ros2_ws/track-runs`. Keep the map and route together. Check the
route on `/track_learning/learned_path` against `/map` for wall clearance,
feasible turns, and a continuous loop. Check that the rover and scan align with
the map. If you see an unsafe route, cancel the driver and leave the stop latched.

When the driver prints `Learned route ready`, confirm the rover is stationary
and the course is clear. Follow [Stop and reset](#stop-and-reset) to clear all
three latches, then repeat the enable sequence to begin replay. The driver
allows up to three minutes for each arm. It latches a stop when the requested
laps finish and writes `run.json` with lap timing. Unexpected stops, lost inputs
or the loss of a safe path end the hardware run. Restart the mapping launch with
a fresh map and use a new output directory to start another session.

## Stop and reset

In the dedicated, sourced autonomy ROS stop terminal, run this command to stop
the rover. It latches all three guards. The software stop releases motor current
and requests steering center, so the rover coasts. Keep the independent physical
propulsion cutoff available throughout the session.

```bash
ros2 topic pub --rate 20 --wait-matching-subscriptions 0 \
  /drive/stop std_msgs/msg/Bool '{data: true}'
```

Leave the stop publisher running until the rover is stationary. Verify
`stop_latched: true` and `enabled: false` in the JSON on `/autonomy/status`,
`/drive/status`, and `/steering/status`. Ctrl+C ends the stop publisher and
leaves the latches set. To reset after standstill, first end any stop publisher,
then clear the input and reset all three latches. Verify the service responses.
The guards stay disabled until separately enabled.

```bash
ros2 topic pub --once --wait-matching-subscriptions 3 --max-wait-time-secs 3 --keep-alive 1.0 \
  /drive/stop std_msgs/msg/Bool '{data: false}'
ros2 service call /autonomy/reset_stop std_srvs/srv/Trigger '{}'
ros2 service call /drive/reset_stop std_srvs/srv/Trigger '{}'
ros2 service call /steering/reset_stop std_srvs/srv/Trigger '{}'
```

To cancel the driver from Windows, detach from the driver window with **Ctrl+B**
then **D**, and send Ctrl+C to it over SSH. A Wi-Fi disconnect leaves the
onboard driver running and blocks remote stop requests. Test this cancellation
with the wheels raised and use the independent cutoff on the track.

```powershell
ssh JETSON_USER@JETSON_HOST "tmux send-keys -t track-learner:driver C-c"
```

## Use Foxglove on Windows

For the manual dashboard's USB-free preview, run
`bash tools/start_manual_dashboard.sh --preview` from the ECD726-CAR root on the
Jetson host. The helper uses a separate ROS domain and starts without a USB
device mapping. Package the slider with `python3 tools/package_erpm_slider.py`
in a second host terminal. Import the manual layout in Foxglove, verify
`mode: preview` on `/manual/status`, and check the red **STOP** control. End the
manual dashboard session before starting track learning.
[Manual preview and Windows setup](OVAL_AUTONOMY.md#manual-preview-and-bench-controls)

For live track learning, run a separate read-only bridge in an autonomy ROS
terminal. Use the SSH stop terminal or the physical cutoff to stop the car.

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p address:=0.0.0.0 -p port:=8766 \
  -p 'capabilities:=[connectionGraph]' \
  -p 'topic_whitelist:=["^/(autonomy|drive|steering)/status$", "^/(scan|odom|map|tf|tf_static)$", "^/track_learning/(learned_path|local_path)$"]'
```

On the Jetson host, get the container IP for the default Docker bridge network.
Use it in place of `CONTAINER_IP` in the Windows command and keep the tunnel open.

```bash
docker inspect --format '{{(index .NetworkSettings.Networks "bridge").IPAddress}}' \
  vesc-firmware-tools
```

```powershell
ssh -NT -L 127.0.0.1:8766:CONTAINER_IP:8766 JETSON_USER@JETSON_HOST
```

Open **Foxglove WebSocket** at `ws://127.0.0.1:8766`. In **3D**, select `map`
as the display frame and show `/map`, `/scan`, and `/track_learning/learned_path`.
Use **Raw Messages** to check the three status topics.
[Foxglove ROS 2 connection](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)
