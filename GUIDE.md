# Drone launcher app (start here)

```bash
~/drone_px4_isaac/start_app.sh
```

One window that starts everything and shows the drone camera with YOLO detection boxes:

1. **Start simulation** launches Isaac Sim with the Pegasus quadcopter, PX4 SITL (auto-started, lockstep), the
   Baylands park scene, a red hatchback and a standing person ~11 to 15 m in front of the drone.
   Isaac Sim needs about a minute to load. Tick "Hide Isaac Sim window" to run it headless (faster).
2. **Open QGroundControl**, wait for "Ready To Fly", click **Takeoff**, then fly with the virtual joystick
   (QGC: Application Settings > General > Fly View > Virtual Joystick).
3. The camera feed (`/drone/camera/image_raw`) shows in the app with `models/best.pt` (YOLOv8s fine-tuned on
   VisDrone: pedestrian, people, car, van, truck, ...). Detection on/off and confidence are in the side panel.
4. **Stop all** (or closing the window) stops Isaac Sim, PX4 and QGC.

| Path | What |
|---|---|
| `app.py`, `start_app.sh` | The launcher (PyQt5). `start_app.sh` sources ROS 2 Humble and uses `venv/` |
| `venv/` | Python 3.10 env with torch (CUDA), ultralytics, PyQt5, OpenCV (was `~/ml_project/myenv`) |
| `models/best.pt` | Detector (copy of `ml_project/best8s.pt`, the VisDrone YOLOv8s) |
| `assets/pegasus.usd` | Drone (Pegasus layout, 1.5 kg quad-X) |
| `assets/baylands/` | Baylands park converted from the Gazebo Fuel model (OpenRobotics) with ground colliders |
| `assets/car/`, `assets/person/` | Gazebo Fuel "Hatchback red" and "Standing person" converted to USD |
| `assets/src/` | The original Fuel downloads; `tools/mesh_to_usd.py` and `tools/add_colliders.py` rebuild the USDs |

### Drone models and gimbal (added 2026-10-13)

| App choice | USD | PX4 airframe | Notes |
|---|---|---|---|
| Pegasus quadcopter | `assets/pegasus.usd` | `isaac_iris` (22000) | 1.5 kg |
| Holybro X500 | `assets/x500/x500.usd` | `isaac_x500` (22001) | built from PX4's gz `x500` meshes/mass by `tools/build_drones.py x500` |
| FPV 5-inch racer | `assets/fpv/fpv.usd` | `isaac_fpv` (22002) | 0.6 kg, primitives (`tools/build_drones.py fpv`), scaled-down rate gains |
| 3DR Iris | Pegasus stock | `isaac_iris` | |

The airframes live in `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/2200*_isaac_*` (also copied into
`build/px4_sitl_default/etc/...`, so no PX4 rebuild is needed). They set `SIM_BAT_MIN_PCT 100` so the simulated battery
never drains into a failsafe.

Gimbal: the camera sits on `/World/drone/body/gimbal` (yaw, then pitch; + pitch = down). The sim listens on
UDP 127.0.0.1:14600 for `{"pitch": deg, "yaw": deg}` and slews at 60 deg/s (pitch -30..90, yaw +-170). The app's
gimbal panel (sliders, presets, keys W/S/A/D, C) sends these.

Same scene without the app: `./run_sim.sh --usd assets/pegasus.usd --env baylands --objects --cam-tilt 35`
(`--env ground` for the plain ground plane, `--cam-mount down` for a downward camera).

Tested 2026-10-13 (headless): PX4 armed, took off to 7.2 m, hovered and landed in Baylands; the camera stream ran at
~13 FPS in the app and the detector found the person in every frame and the car in some frames (the car is a small,
low-poly model seen at a shallow angle; it is detected more often from higher up or with the downward camera).

---

# Custom drone + PX4 SITL + QGroundControl + ROS 2 camera in Isaac Sim

Everything here targets what is **installed on this PC** (checked 2026-10-12), not Isaac Sim 4.x / 2023.x:

| Component | Installed | Where |
|---|---|---|
| NVIDIA Isaac Sim | **6.0.0-rc.59** (Python 3.12) | `~/isaacsim` (the identical copy was moved to `~/Not needed Now`) |
| Pegasus Simulator | **6.0.0** (`fcb99c0`), pip-installed (editable) into Isaac Sim's Python | `~/PegasusSimulator` |
| PX4-Autopilot | **v1.14.3**, `px4_sitl_default` already built | `~/PX4-Autopilot`  |
| ROS 2 | Humble `ros-base` + `image_transport` (no `cv_bridge`, no `rqt_image_view`) | `/opt/ros/humble` |
| QGroundControl | AppImage | `~/drone_px4_isaac/assets/QGroundControl.AppImage` |
| GPU | RTX 4060 8 GB | |

(2026-10-13: `~/isaac sim` was moved to `~/Not needed Now/isaac sim`; its `pegasus.usd` and QGC AppImage now live
in `assets/`.) The `~/isaac sim` folder held a second Pegasus checkout, the v1.18 PX4 tree, an older `my_custom_drone.py`
(identical to `~/my_custom_drone.py`), `pegasus.usd` (a valid Pegasus-layout drone, 1.5 kg) and `copter.usd`
(a full scene that references a missing `~/Downloads/iris.usd`, **not** a spawnable drone).

Files in this folder:

| File | Purpose |
|---|---|
| `launch_custom_drone.py` | The standalone Isaac Sim app (spawn drone, PX4 bridge, camera, ROS 2 / RTSP, main loop) |
| `run_sim.sh` | Wrapper: sets Isaac Sim's internal ROS 2 Humble env, warns about stale PX4, runs the script |
| `drone_usd_check.py`, `check_drone_usd.sh` | Validates a custom drone USD against what Pegasus needs (no GPU needed) |
| `view_camera.py` | ROS 2 viewer / FPS meter / frame grabber for `/drone/camera/image_raw` (no `cv_bridge` needed) |
| `mavlink_smoke_test.py` | Scripted arm → takeoff → hover → land over UDP 14540 (same thing you do in QGC) |
| `logs/` | Logs and camera frames from the test runs |

---

## 1. Architecture and ports

```
 ┌────────────── Isaac Sim 6 (launch_custom_drone.py) ──────────────┐
 │ PhysX 250 Hz ─ Pegasus Multirotor (/World/drone/body, rotor0..3) │
 │   sensors: IMU, GPS, baro, mag ──► PX4MavlinkBackend  TCP :4560  │◄──── PX4 SITL (px4 -i 0, lockstep)
 │   rotor forces ◄── HIL_ACTUATOR_CONTROLS                         │        │  UDP 14550 ──► QGroundControl
 │ RTX camera /World/drone/body/onboard_camera                      │        │  UDP 14540 ──► MAVSDK / MAVROS / smoke test
 │   OmniGraph ROS2CameraHelper ──► /drone/camera/image_raw         │
 │                                 /drone/camera/camera_info, /clock│
 │   (optional) RTSPCameraHelper ──► rtsp://localhost:8554/drone_cam │
 └──────────────────────────────────────────────────────────────────┘
```

| Port | Proto | Who listens | Who sends | Purpose |
|---|---|---|---|---|
| 4560 + id | TCP | Isaac Sim (Pegasus, `tcpin`) | PX4 `simulator_mavlink` | HIL_SENSOR / HIL_GPS ⇄ HIL_ACTUATOR_CONTROLS |
| 14550 | UDP | **QGroundControl** | PX4 (GCS link, local port 18570) | Telemetry + commands |
| 14540 + id | UDP | Offboard client (MAVSDK, MAVROS, `mavlink_smoke_test.py`) | PX4 (onboard link, local port 14580) | Offboard API |
| 8554 | TCP/RTSP | Isaac Sim (`--rtsp-port`) | | H.264 camera stream |

Only one process may bind each port. QGC owns 14550; never point another tool at `udpin:14550` while QGC is open.

---

## 2. Environment and prerequisites

### 2.1 System packages (already present except where marked)

```bash
# Build deps for PX4 (already satisfied: px4_sitl_default is built)
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# ROS 2 image tooling (NOT installed yet; needed only for rqt_image_view / cv_bridge users)
sudo apt install ros-humble-cv-bridge ros-humble-rqt-image-view ros-humble-image-transport-plugins

# QGroundControl runtime deps (GStreamer for QGC's video widget, FUSE for the AppImage)
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libfuse2
sudo usermod -aG dialout "$USER"     # then log out/in; also: sudo apt remove modemmanager -y

# pymavlink for the system python (already installed)
python3 -m pip install --user pymavlink
```

### 2.2 Isaac Sim 6.0

Already installed at `~/isaacsim`. `~/.bashrc` exports `ISAACSIM_PATH=$HOME/isaacsim` and
`ISAACSIM_PYTHON=$ISAACSIM_PATH/python.sh`. Isaac Sim 6 ships its **own** ROS 2 Humble and Jazzy libraries
under `exts/isaacsim.ros2.core/{humble,jazzy}` because it runs Python 3.12 while Ubuntu 22.04's Humble is Python 3.10.
So:

* **Do not** `source /opt/ros/humble/setup.bash` in the terminal that launches Isaac Sim.
* Do set `ROS_DISTRO=humble`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and add
  `~/isaacsim/exts/isaacsim.ros2.core/humble/lib` to `LD_LIBRARY_PATH`. `run_sim.sh` does exactly this
  (it mirrors `~/isaacsim/setup_ros_env.sh`).
* Keep the same `ROS_DOMAIN_ID` (default 0) in the Isaac Sim terminal and in every ROS terminal.

### 2.3 PX4-Autopilot SITL (headless target)

PX4 is built for SITL without Gazebo. To rebuild after a pull or a clean:

```bash
cd ~/PX4-Autopilot
git describe --tags            # v1.14.3
make px4_sitl_default          # builds build/px4_sitl_default/bin/px4, no simulator is started
```

For PX4 ≥ v1.14 the airframe name is `gazebo-classic_iris` (SYS_AUTOSTART 10015). For PX4 < v1.14 it is `iris`
(`--px4-airframe iris`). Pegasus' launcher starts PX4 with `PX4_SIM_MODEL=gazebo-classic_iris`, rootfs in a temp dir.

### 2.4 Pegasus Simulator 6.0

Installed as an editable package inside Isaac Sim's Python, which is what makes `import pegasus.simulator` work in
standalone scripts:

```bash
cd ~/PegasusSimulator/extensions
$ISAACSIM_PYTHON -m pip install --editable pegasus.simulator
$ISAACSIM_PYTHON -m pip show pegasus-simulator     # Version: 6.0.0, Location: ~/PegasusSimulator/...
```

Pegasus reads `~/PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml`
(`px4_dir: ~/PX4-Autopilot`, `px4_default_airframe: gazebo-classic_iris`, home = Lisbon, 90 m AMSL).
`launch_custom_drone.py` passes these explicitly (`--px4-dir`, `--px4-airframe`) so the YAML is not required.

### 2.5 ROS 2 Humble consumer side

```bash
source /opt/ros/humble/setup.bash
ros2 topic list            # /clock /drone/camera/camera_info /drone/camera/image_raw
```

### 2.6 QGroundControl

```bash
chmod +x ~/isaac\ sim/QGroundControl.AppImage
~/isaac\ sim/QGroundControl.AppImage
```

QGC defaults are correct: **Application Settings → Comm Links → AutoConnect → UDP** enabled; QGC listens on UDP 14550.
If you disabled autoconnect earlier, add a link manually: *Comm Links → Add → Type UDP, Port 14550, Server
127.0.0.1:18570* and connect it. Recommended for sim: *Application Settings → General → Offline vehicle
firmware type PX4, Units metric*, and *Fly View → Virtual Joystick* on if you have no RC/gamepad.

---

## 3. The script: `launch_custom_drone.py`

Run it through the wrapper (`./run_sim.sh ARGS`) or directly with `$ISAACSIM_PYTHON launch_custom_drone.py ARGS`
after setting the ROS env from 2.2. What it does, in order:

1. Parses arguments, then starts `SimulationApp` (GUI unless `--headless`; renderer `RaytracedLighting`).
2. Enables `isaacsim.ros2.bridge` (and `isaacsim.streaming.rtsp` with `--rtsp-port`).
3. Builds the world: Z-up, 1 m units, a PhysicsScene (gravity 9.81, CPU PhysX, MBP broadphase), a 500 m ground
   plane, sun + dome light, five landmark boxes. `--env "Default Environment"` (or any Pegasus/NVIDIA environment
   name or `.usd` path) loads that instead; NVIDIA environments download from NVIDIA's S3.
4. Sets physics to 250 Hz **after** the physics scene exists (Pegasus' PX4 setting).
5. Validates the drone USD (`drone_usd_check.py`), computes the spawn height so the drone **rests** on the ground,
   and spawns it at `/World/drone` with a Pegasus `Multirotor` (with cached rotor force handles, about 1.8x faster).
6. Attaches `PX4MavlinkBackend`: `tcpin` on 4560+id, lockstep, PX4 auto-launched (`--no-px4-autolaunch` to start it
   yourself).
7. Creates `/World/drone/body/onboard_camera` (a USD camera parented to the body rigid body): forward
   (`rotateXYZ(90-tilt, 0, -90)`, offset 0.15/0/0.05 m) or down (`rotateXYZ(0, 0, -90)`, offset 0/0/-0.08 m, image-up =
   nose). Focal length is derived from `--hfov` with the 20.955 mm aperture; vertical aperture keeps square pixels.
8. Builds one OmniGraph: `OnTick → IsaacCreateRenderProduct(640x480) → ROS2CameraHelper(rgb)` +
   `ROS2CameraInfoHelper` + `ROS2PublishClock`, and optionally `RTSPCameraHelper`. The RGB/info publishers are gated to
   `round(60 / fps)` rendered frames.
9. Plays the timeline and loops `simulation_app.update()` until the window closes, Ctrl+C / SIGTERM, or
   `--max-seconds`; then stops the timeline, kills PX4, closes the app.

Main options:

```
--usd PATH            custom drone USD (default /path/to/my_drone.usd); 'iris' / 'pegasus' = stock models
--cam-mount forward|down   --cam-tilt DEG   --cam-offset X Y Z
--width 640 --height 480 --fps 30 --hfov 90
--ros-namespace drone   (topics /drone/camera/image_raw, /drone/camera/camera_info)
--rtsp-port 8554        (rtsp://localhost:8554/drone_cam, H.264)
--env ground|"Default Environment"|path.usd
--px4-dir ~/PX4-Autopilot  --px4-airframe gazebo-classic_iris  --no-px4-autolaunch  --vehicle-id 0
--rotor-constant 8.54858e-6  --max-rotor-vel 1100   (thrust curve for heavier/lighter frames)
--headless  --max-seconds N  --debug-state  --no-ros  --no-fast-forces
```

---

## 4. Execution sequence

### 4.1 Pre-flight checks (once per new drone USD)

```bash
cd ~/drone_px4_isaac
./check_drone_usd.sh /path/to/my_drone.usd      # must end with "RESULT : OK for Pegasus"
pgrep -ax px4 && pkill -x px4                    # no stale PX4 from an earlier crash
```

### 4.2 Run it

Terminal 1, QGroundControl:

```bash
~/isaac\ sim/QGroundControl.AppImage
```

Terminal 2, Isaac Sim + PX4 (PX4 is started automatically when the timeline plays):

```bash
cd ~/drone_px4_isaac
./run_sim.sh --usd /path/to/my_drone.usd --cam-mount forward --fps 30
# try it right now with your existing model:
./run_sim.sh --usd assets/pegasus.usd
```

Wait for `Simulation playing` in the console. PX4 boots and its console lines (`INFO [px4] Startup script returned
successfully`, `home set`) appear in the same terminal.

### 4.3 (Alternative) start PX4 yourself

```bash
./run_sim.sh --usd /path/to/my_drone.usd --no-px4-autolaunch      # terminal 2: Isaac Sim waits on TCP 4560
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gazebo-classic_iris ./build/px4_sitl_default/bin/px4   # terminal 3
```

Use this when you want PX4's interactive `pxh>` shell (`param set ...`, `commander status`, `listener vehicle_attitude`).

### 4.4 Verify the telemetry handshake in QGC

Within a few seconds QGC's toolbar shows the vehicle:

* **Artificial horizon** level and the compass pointing **East (≈90°)**: the drone spawns with yaw 0 in ENU, i.e. its
  nose points East. Tilting is shown live when it flies.
* **GPS**: 3D fix, 10 satellites, home at 38.7368 N, -9.1380 E, 90 m (Pegasus' default, change in `configs.yaml`).
* **Flight mode**: *Hold* / *Position*; status turns to **Ready To Fly** after the EKF converges (about 5 to 10 s
  of *sim* time; the sim runs slower than real time, see 5.4).
* **Analyze Tools → MAVLink Inspector** should show `HIGHRES_IMU`, `ATTITUDE`, `GPS_RAW_INT` streaming.

### 4.5 Arm, take off, fly waypoints

1. Click **Ready To Fly → Arm** (slide to confirm), or the *Takeoff* action in the left toolbar, set altitude (e.g.
   5 m), slide to confirm. PX4 arms and climbs.
2. **Go to location**: click on the map → *Go to location* → slide. The drone flies there in Position/Hold.
3. **Mission**: *Plan* view → *Takeoff* item → click to add waypoints (set altitude 5 to 10 m) → *Return* item →
   **Upload** → back to *Fly* → *Start Mission* slider.
4. **Land / RTL**: *Land* or *Return* in the left toolbar. PX4 disarms automatically after touchdown.
5. Manual flight: enable the virtual joystick (or a gamepad: *Vehicle Setup → Joystick*) and switch to *Position*.

Scripted equivalent (UDP 14540, can run while QGC is open):

```bash
python3 mavlink_smoke_test.py --alt 5 --hover 10
```

### 4.6 Verify the camera stream

ROS 2 (separate terminal, system Python):

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /drone/camera/image_raw            # rate is in wall-clock Hz, see 5.4
ros2 topic echo --once /drone/camera/camera_info
python3 ~/drone_px4_isaac/view_camera.py          # OpenCV window, prints FPS; no cv_bridge needed
python3 ~/drone_px4_isaac/view_camera.py --no-window --seconds 5 --save-frame /tmp/frame.png
ros2 run rqt_image_view rqt_image_view /drone/camera/image_raw   # after: sudo apt install ros-humble-rqt-image-view
```

OpenCV with cv_bridge (after installing `ros-humble-cv-bridge`), the minimal subscriber is:

```python
import rclpy, cv2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n = rclpy.create_node("cam"); b = CvBridge()
n.create_subscription(Image, "/drone/camera/image_raw",
    lambda m: (cv2.imshow("cam", b.imgmsg_to_cv2(m, "bgr8")), cv2.waitKey(1)), qos_profile_sensor_data)
rclpy.spin(n)
```

RTSP (start the sim with `--rtsp-port 8554`):

```bash
ffplay -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/drone_cam      # or VLC
gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:8554/drone_cam latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

(`avdec_h264` comes from `gstreamer1.0-libav`, not installed yet.) QGC can show it too:
*Application Settings → Video → Source RTSP Video Stream, URL rtsp://127.0.0.1:8554/drone_cam*.

---

## 5. Troubleshooting and best practices

### 5.1 MAVLink connection drops and port conflicts

| Symptom | Cause | Fix |
|---|---|---|
| `OSError: [Errno 98] Address already in use` at start | Another Isaac Sim/Pegasus instance already holds TCP 4560 | Close it (`pgrep -af launch_custom_drone`), then retry. Reproduced in testing. |
| PX4 prints `poll timeout 0, 25` / `PX4 server not running` | PX4 started but nothing listens on 4560 (sim not playing, or a stale PX4) | `pkill -x px4`, press Play / restart the script. PX4 must start **after** the sim listens (autolaunch does this). |
| QGC shows nothing | QGC autoconnect UDP off, or another program bound UDP 14550 | Re-enable autoconnect; `ss -lunp \| grep 14550` must show QGC only. Don't run MAVROS/MAVSDK on 14550, use 14540. |
| Two vehicles / flickering vehicle in QGC | A leftover PX4 instance from a crashed run | `pkill -x px4`. `run_sim.sh` warns when one is running. |
| MAVROS cannot connect | Wrong URL | `fcu_url:=udp://:14540@127.0.0.1:14580` |
| Multiple drones | Port collisions | Give each a different `--vehicle-id N`: TCP 4560+N, offboard UDP 14540+N, QGC still 14550 (MAV_SYS_ID = N+1). |
| Link drops when the sim stalls | Lockstep: PX4 time only advances with sim time, so QGC shows "communication lost" while Isaac Sim is loading/compiling shaders | Wait; it recovers. Don't disable lockstep (`enable_lockstep`), that causes EKF drift instead. |
| `pxh>` closes / zombie PX4 after Ctrl+C | Script killed with SIGKILL | Use Ctrl+C or close the window; the script stops PX4 itself (verified). Otherwise `pkill -x px4`. |

### 5.2 Custom `.usd` multirotor: structure, colliders, rigid bodies

Pegasus applies thrust per rotor prim and torque to the body, so your USD must have (names exact):

```
<defaultPrim>          Xform  + PhysicsArticulationRootAPI
├── body               Xform  + RigidBodyAPI + MassAPI (+ colliders below it)   FLU frame: X fwd, Y left, Z up
├── rotor0 … rotor3    Xform  + RigidBodyAPI + MassAPI (small mass, NO colliders)
│   └── jointN         PhysicsRevoluteJoint body → rotorN, axis Z (visual spin only)
```

PX4 quad-X motor order (Iris airframe): **rotor0 front-right (CCW), rotor1 rear-left (CCW), rotor2 front-left (CW),
rotor3 rear-right (CW)** — matches `rot_dir [-1, -1, 1, 1]`. Swapped rotors = instant flip on takeoff.

Rules that prevent instability and sensor drift:

* **Units and axes**: stage `metersPerUnit = 1`, `upAxis = Z`. A cm-unit export makes the mass/thrust ratio 100x off.
* **Explicit mass and inertia** on body and rotors (MassAPI). PhysX-from-density masses are almost always wrong.
* **Thrust-to-weight ≥ 1.6–2**. The default thrust curve gives about 41 N total; `check_drone_usd.sh` prints T/W. For a
  heavier frame raise `--rotor-constant` (T = k·ω²) or `--max-rotor-vel`.
* **Simple convex colliders** (boxes/capsules for arms and legs) on `body` only; disable collisions on propellers.
  Concave triangle-mesh colliders on dynamic bodies are invalid in PhysX.
* **Spawn at rest**. If the drone free-falls even 2 cm when the sim starts, PX4's EKF aligns its tilt from ~0 g
  accelerometer samples and can initialise **upside down**: QGC then shows *"Preflight Fail: Attitude failure (roll)",
  "High Gyro Bias", "velocity estimate error"* and refuses to arm. This happened with `pegasus.usd` at Pegasus' default
  spawn height of 0.07 m (its feet are 0.046 m below the origin). The script now computes the resting height from the
  USD bounding box. If you override `--spawn`, keep Z at the printed resting height.
* **Physics rate**: 250 Hz, set after the physics scene exists (done). At 60 Hz the IMU stream is too coarse for PX4.
* **No camera physics**: the camera is a plain USD prim under `body`; never give sensor prims RigidBodyAPI.
* **Rotor joint drives**: no stiffness/damping on rotor joints, or they fight the prop-spin animation.
* `--debug-state` prints ground-truth attitude/rates and the raw HIL_SENSOR values PX4 receives; use it whenever PX4
  reports estimator errors. A still drone must show ~(0, 0, -9.81) accel and ~0 gyro.

### 5.3 Camera tips

* Forward FPV tilt: `--cam-tilt 20`. Downward mapping camera: `--cam-mount down` (image top = nose).
* Higher resolutions cost render time 1:1 on an 8 GB RTX 4060; keep 640x480 for control loops.
* The ROS image is `rgb8`; use `bgr8` conversion for OpenCV. QoS: subscribe with `qos_profile_sensor_data`.
* Use `/clock` with `use_sim_time:=true` in ROS nodes so image stamps line up with sim time.

### 5.4 Real-time factor

With the drone, PX4 lockstep at 250 Hz and the camera rendering, this PC runs the simulation at roughly
**0.25–0.5x real time** (measured: about 0.5x with the cached rotor forces, about 0.28x with stock Pegasus force code,
camera publishing about 6–7 Hz wall-clock = about 25–30 Hz sim time). Lockstep keeps PX4 consistent, so flight is
correct but looks slow-motion, and wall-clock rates (`ros2 topic hz`) read lower than `--fps`. Headless (`--headless`)
and smaller camera resolutions help.

---

## 6. What was verified on this PC (2026-10-12)

Verified by running it (headless, `~/isaac sim/pegasus.usd`, logs in `logs/`):

* USD checker on `pegasus.usd` (OK, 1.504 kg, T/W 2.80) and `copter.usd` (correctly rejected).
* Isaac Sim 6 + Pegasus 6 spawn, PX4 v1.14.3 auto-launch, TCP 4560 lockstep link, GPS 3D fix.
* Arm → takeoff to 5 m (reached 4.4 m on the EKF altitude) → 60 s hover → land → disarm, via MAVLink on UDP 14540
  (`mavlink_smoke_test.py`), for both forward and downward camera configurations.
* `/drone/camera/image_raw` (640x480 rgb8, frame `onboard_camera`), `/drone/camera/camera_info`, `/clock` visible
  from system ROS 2 Humble; frames grabbed in flight: `logs/frame_fwd.png`, `logs/frame_down.png`.
* RTSP server on 8554 delivering an H.264 RTP stream (RTSP handshake + packets with GStreamer; not decoded, no H.264
  decoder installed).
* Clean shutdown: PX4 stopped on Ctrl+C/SIGTERM and on `--max-seconds`.

Written but not run by me:

* GUI mode (only headless was run) and clicking through QGroundControl itself; QGC uses the same PX4 link that the
  scripted flight used, on UDP 14550.
* `--no-px4-autolaunch` with a manually started PX4, `--env` with NVIDIA's downloadable environments,
  `rqt_image_view` and the cv_bridge snippet (packages not installed), multi-vehicle `--vehicle-id`.
