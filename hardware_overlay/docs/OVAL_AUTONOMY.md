# Physical oval bring-up

The oval uses ROS 2 Jazzy on the Jetson. Nav2 Collision Monitor publishes checked
velocity commands. The autonomy adapter turns them into forward ERPM and steering
commands for the existing motor and steering guards. All three start disabled,
and the hardware Nav2 profile caps forward velocity at 0.15 m/s.

LiDAR, odometry, localization, planning, and control all run on the Jetson. The
laptop can monitor and send stop requests over the network. Simulation and ROS
preview cover software behavior. Verify the integrated stack and stopping
distance on the physical car before track testing.

## Jetson Orin Nano setup

Keep the working JetPack installation on an existing Jetson. For a fresh kit,
follow NVIDIA's instructions for your module, boot firmware and storage. Install
Docker Engine for the Jetson's host Ubuntu release. The repository's Docker
images provide the ROS 2 Jazzy environment. Run the container commands on the
Jetson, with Git, Python 3, `tmux` and `fuser` available on the host.
[NVIDIA Orin Nano setup](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_bsp.html)
[Docker Engine for Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

From the root of the existing ECD726-CAR checkout on the Jetson, clone the
autonomy branch and apply its hardware overlay. The existing checkout is the
workspace mounted at `/work` in the VESC Tool container.

```bash
git clone --branch feature/jetson-oval-autonomy --single-branch \
  https://github.com/Jay947462/ecd726-sim.git src/ecd726-sim
bash src/ecd726-sim/hardware_overlay/apply.sh "$PWD"
python3 tools/fetch_dependencies.py
```

If the autonomy checkout already exists, use its updated branch and run the
overlay command from the ECD726-CAR workspace. For a new Jetson, follow
`docs/VESC_TOOL_SETUP.md` in the ECD726-CAR checkout. Preserve an existing
working `vesc-firmware-tools` container and verify that it mounts this checkout
and maps this controller. The tool is used for controller identity and firmware
watchdog preflight, then releases the serial port for the ROS drive node.

Install runtime packages in the existing container and build the workspace
before any control session. Run the following commands on the Jetson host.

```bash
docker start vesc-firmware-tools
docker exec --user root vesc-firmware-tools sh -ec \
  'test -e /ros2_ws || ln -s /work /ros2_ws'
docker exec --user root vesc-firmware-tools bash -ec '
  apt-get update
  apt-get install -y python3-colcon-common-extensions python3-pytest \
    ros-jazzy-serial-driver ros-jazzy-xacro ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander ros-jazzy-foxglove-bridge'
docker exec -it vesc-firmware-tools bash
```

Inside the container, build the VESC, drive and oval packages. The LiDAR and
odometry sources have their own installation and calibration.

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-select \
  vesc_msgs vesc_driver car_drive ecd726_description \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
python3 -m pytest -q src/car_drive/test/test_autonomy_preview.py
```

The test exercises the guards with ROS preview outputs. For the following
sections, open each autonomy ROS terminal from the Jetson host and source the
same installation inside it.

```bash
docker exec -it -e ROS_DOMAIN_ID=77 vesc-firmware-tools bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

## Stop and reset

Keep a terminal ready inside the drive container. Source ROS and the workspace,
and use the drive session's ROS domain. The autonomy helper defaults to domain 77.

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 topic pub --rate 20 --wait-matching-subscriptions 0 \
  /drive/stop std_msgs/msg/Bool '{data: true}'
```

Leave the publisher running until the car is stopped. Check
`stop_latched: true` and `enabled: false` in the JSON on
`/drive/status`, `/steering/status`, and `/autonomy/status`.
Ctrl+C ends the publisher and leaves the latches set. The software stop releases
motor current and requests steering center. The car coasts. Keep an independent
physical propulsion cutoff available for every powered test.

With the car stationary and the stop publisher ended, release the stop input
and reset each latch. Verify the service responses. The guards remain disabled.

```bash
ros2 topic pub --once --wait-matching-subscriptions 3 --max-wait-time-secs 3 --keep-alive 1.0 \
  /drive/stop std_msgs/msg/Bool '{data: false}'
ros2 service call /autonomy/reset_stop std_srvs/srv/Trigger '{}'
ros2 service call /drive/reset_stop std_srvs/srv/Trigger '{}'
ros2 service call /steering/reset_stop std_srvs/srv/Trigger '{}'
```

In a drive-and-steering session without the autonomy adapter, use two matching
subscriptions for the reset message and call the drive and steering services.

## Cancel the onboard driver over Wi-Fi

Before running the oval driver, start a dedicated `tmux` window on the Jetson
host. Open the autonomy ROS container shell inside the window and run the driver
there. Press **Ctrl+B**, then **D** to detach.

```bash
tmux new-session -s oval-driver -n driver
```

From the laptop, use the Jetson's SSH account and reachable address. The named
window receives Ctrl+C, causing the driver to request Nav2 goal cancellation.
In armed autonomy the guard also watches for loss of the driver's heartbeat.

```powershell
ssh JETSON_USER@JETSON_HOST "tmux send-keys -t oval-driver:driver C-c"
```

A Wi-Fi disconnect leaves the onboard driver running and prevents remote stop
requests. Check Ctrl+C and the latched stop from a second SSH terminal with the
wheels raised. Keep the physical propulsion cutoff available on the track.
[tmux terminal sessions](https://github.com/tmux/tmux/wiki/Getting-Started#attaching-and-detaching)

## Foxglove on Windows

### Manual preview and bench controls

Before an autonomy session, use the separate manual dashboard to verify controls.
Build and launch the manual preview from the hardware checkout on the Jetson.
The helper selects a separate ROS domain and starts without a VESC device mapping.
It prints the WebSocket URL. Its default bind address comes from Tailscale.
For a trusted local network without Tailscale, set `MANUAL_DASHBOARD_ADDRESS`
to the Jetson's reachable LAN address.

```bash
bash tools/start_manual_dashboard.sh --preview
python3 tools/package_erpm_slider.py
```

Run the packaging command from a second Jetson terminal. Copy the generated
bundle to Windows, install the local extension, import `manual-controls.json`,
and open **Foxglove WebSocket** with the printed URL. Verify `mode: preview` in
`/manual/status`, then check enable, release, and the red **STOP** button. For
the manual hardware stage, use the wheels-raised procedure in
`dashboards/README.md` in your ECD726-CAR checkout. The Windows layout guide
is in `dashboards/INSTALL-WINDOWS.md` in the same checkout. The manual
controller must exit before the onboard autonomy session opens the VESC.

### Read-only autonomy monitoring

The autonomy launch runs independently of Foxglove. For optional monitoring
on the same Jetson, run a separate bridge in an autonomy ROS terminal. The
bridge exposes telemetry and the ROS connection graph while command publishing
and service calls stay disabled.

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p address:=0.0.0.0 -p port:=8766 \
  -p 'capabilities:=[connectionGraph]' \
  -p 'topic_whitelist:=["^/(autonomy|drive|steering)/status$", "^/drive/(target_erpm|vesc/sensors/core)$", "^/(scan|odom|map|amcl_pose|tf|tf_static)$", "^/oval/reference_path$"]'
```

For the default VESC Tool Docker bridge network, get the private container
address from the Jetson host. Use the resulting address in place of
`CONTAINER_IP` in the Windows command. Keep the SSH tunnel open.

```bash
docker inspect --format '{{(index .NetworkSettings.Networks "bridge").IPAddress}}' \
  vesc-firmware-tools
```

```powershell
ssh -NT -L 127.0.0.1:8766:CONTAINER_IP:8766 JETSON_USER@JETSON_HOST
```

In Foxglove, open **Foxglove WebSocket** at `ws://127.0.0.1:8766`.
Use **Raw Messages** for `/autonomy/status`, `/drive/status`, and
`/steering/status`. In **3D**, choose `map` as the display frame and display
the map, scan and planned path. Check `/amcl_pose` and the pose overlay before
arming. Use the Jetson stop terminal to stop the car.
[Foxglove ROS 2 connection](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)
[Foxglove Bridge capabilities](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md#capabilities)

## Prepare the physical inputs

- Follow the **Motor hardware bring-up** in `src/car_drive/README.md` in the
  ECD726-CAR checkout.
  Check the VESC firmware command timeout on this controller. With the wheels
  raised and clear, verify software stop, command loss, driver exit, and USB loss.
  Observe ERPM and motor current as well as the ROS status. Verify the independent
  cutoff before moving to the floor.
- Measure the speed conversion in ERPM per m/s and both effective wheel angles
  at the calibrated steering endpoints. Use positive angle magnitudes in radians.
  The current controller profile assumes a minimum turn radius of 0.65 m. Check
  both measured turning radii, the vehicle footprint and the steering response
  under load before following a path. Fill in
  a local copy of [autonomy.hardware.yaml](../src/car_drive/config/autonomy.hardware.yaml)
  with the measured values and use a separately tested forward ERPM ceiling. Save
  the copy as `src/car_drive/config/autonomy.local.yaml` in the hardware
  workspace. The shipped template contains zeros that prevent launch.
- Publish real, wall-clock `/scan` and calibrated `/odom`. Provide the measured
  LiDAR transform and the `odom → base_link` transform. AMCL and Collision Monitor
  also use `base_footprint`. Verify the transform chain and scan overlay in RViz
  while stationary and during manual movement. Install and activate an appropriate
  driver for the actual Hokuyo and its configured Ethernet address.
  [Hokuyo ROS 2 driver](https://github.com/Hokuyo-aut/urg_node2)
- Map permanent track features and mark the start. The current oval planner
  expects two 6 m centerline straights and two semicircles of 3 m centerline radius.
  The lane is about 1.29 m wide, and the start is at the midpoint of a straight
  at map pose `(0, 2, 0)` in metres and radians. Measure the actual course and
  align its map to those coordinates. The oval geometry in the driver needs an
  update if the physical course uses different dimensions. Use walls or barriers
  observable at LiDAR height for localization and leave movable cones out of
  the static map.
- Use a closed, level area. With a spotter and the independent cutoff ready,
  measure the full stop time and distance at the intended speed on the actual
  surface. Start with a clear straight, then one empty lap, then add a soft cone
  after checking tracking and clearance.

## Launch after the bench checks

From the hardware checkout on the Jetson host, start the guard session. Set
`CHECKED_FORWARD_ERPM` to the positive cap measured on this car. Keep the manual
dashboard session closed and use the same ROS domain for the sensors and Nav2.

```bash
STEERING_AUTONOMY_CONFIG=src/car_drive/config/autonomy.local.yaml \
STEERING_MAX_ERPM="${CHECKED_FORWARD_ERPM:?Set the bench-tested forward cap}" \
  bash tools/start_steering.sh --hardware
```

The same helper accepts `--preview` for a session with preview motor and steering
topics. Start the physical sensors and odometry separately with wall time.
Launch Nav2 with the measured map in the same ROS domain.

```bash
ros2 launch ecd726_description oval_hardware_nav.launch.py \
  map:="${TRACK_MAP:?Set the absolute physical map YAML path}"
```

Put the car at the measured start. Set the initial pose with RViz, or publish
the following estimate for the measured start `(0, 2, 0)` after AMCL activates.
Verify `map → odom → base_link` and the LiDAR overlay before enabling anything.

```bash
ros2 topic pub --once --keep-alive 1.0 /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {stamp: now, frame_id: map},
    pose: {pose: {position: {x: 0.0, y: 2.0}, orientation: {w: 1.0}},
           covariance: [0.04, 0, 0, 0, 0, 0,
                        0, 0.04, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0.09]}}'
ros2 run tf2_ros tf2_echo map base_link
```

Keep the stop terminal ready. Enable the two lower-level guards while the
autonomy adapter is still disabled and holding zero.

```bash
ros2 service call /steering/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /drive/enable std_srvs/srv/SetBool '{data: true}'
```

Start the oval driver from a second ROS terminal. Use the dedicated `tmux`
window if you want to send Ctrl+C over SSH.

```bash
python3 -B /ros2_ws/src/ecd726-sim/oval_nav2/drive_stable_pass_v2.py \
  --real-time --confirm-measured-oval --direct-reference --laps 1
```

When the driver accepts a path, explicitly enable autonomy from another
ROS terminal. Motion can start after this command succeeds. The adapter
requires fresh checked velocity, scan, odometry, oval driver heartbeat, and
enabled guard status. It only forwards a velocity stamped after arming.

```bash
ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: true}'
```

Loss of a required stream while armed latches the stop. A controller fault or
loss of VESC telemetry disables the existing drive guard and propagates to
autonomy. Recovery requires explicit reset and enable. If the adapter itself
exits, the lower-level command watchdog and the VESC firmware timeout cover
loss of software output. The ROS stop is available to publishers on the same
ROS domain. For a normal end to a run, use the latched stop and then end the
driver and guard sessions.
