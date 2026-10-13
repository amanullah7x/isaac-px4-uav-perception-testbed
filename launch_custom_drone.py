#!/usr/bin/env python
"""
launch_custom_drone.py - Custom multirotor + PX4 SITL + onboard camera -> ROS 2, in NVIDIA Isaac Sim.

Target stack (what is installed on this PC):
    Isaac Sim 6.0.0 (~/isaacsim), Pegasus Simulator 6.0.0 (~/PegasusSimulator), PX4-Autopilot v1.14.3
    (~/PX4-Autopilot, build/px4_sitl_default), ROS 2 Humble (Isaac Sim's internal Humble libs), QGroundControl.

What it does:
    1. Starts SimulationApp (GUI by default).
    2. Builds a physics scene with a ground plane, lights and a few landmark boxes (or loads an environment USD).
    3. Spawns your drone USD through Pegasus' Multirotor class (rigid bodies /body + /rotor0..3, see GUIDE.md).
    4. Attaches the Pegasus PX4 MAVLink backend: Isaac Sim <-> PX4 on TCP 4560 (HIL_SENSOR / HIL_ACTUATOR_CONTROLS),
       PX4 auto-launched in lockstep. PX4 itself then talks to QGC on UDP 14550 and to offboard APIs on UDP 14540.
    5. Creates a camera prim rigidly attached to the drone body (forward or downward, configurable FOV/res/fps).
    6. Publishes RGB + CameraInfo with the Isaac Sim ROS 2 bridge (OmniGraph ROS2CameraHelper) on
       /drone/camera/image_raw and /drone/camera/camera_info, plus /clock.
    7. Steps physics + rendering until the window is closed, Ctrl+C, or --max-seconds, then shuts PX4 down cleanly.

Run (use the wrapper, it sets the ROS 2 environment Isaac Sim needs):
    ./run_sim.sh --usd /path/to/my_drone.usd
or directly:
    $ISAACSIM_PYTHON launch_custom_drone.py --usd /path/to/my_drone.usd
"""

# ----------------------------------------------------------------------------------------------------------------------
# 0. Arguments. Parsed BEFORE SimulationApp so --headless can be honoured. Only stdlib imports are allowed here:
#    every omni/isaacsim/pxr import must happen after SimulationApp() has been created.
# ----------------------------------------------------------------------------------------------------------------------
import argparse
import math
import os
import signal
import sys
import time

DEFAULT_USD = "/path/to/my_drone.usd"  # <-- your custom drone. Override with --usd.

parser = argparse.ArgumentParser(description="Custom drone + PX4 SITL + ROS 2 camera in Isaac Sim (Pegasus)")
parser.add_argument("--usd", default=DEFAULT_USD,
                    help="Custom multirotor USD. Use 'iris' or 'pegasus' for the stock Pegasus models.")
parser.add_argument("--env", default="ground",
                    help="'baylands' (outdoor park, assets/baylands), 'ground' (local ground plane + landmarks, no download), a Pegasus/NVIDIA environment name "
                         "such as 'Default Environment' or 'Curved Gridroom' (downloads from NVIDIA's S3), or a .usd path")
parser.add_argument("--objects", action="store_true",
                    help="Add a car and a person in front of the drone (assets/car, assets/person)")
parser.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
parser.add_argument("--px4-airframe", default="gazebo-classic_iris",
                    help="PX4 SYS_AUTOSTART model name (v1.14+: gazebo-classic_iris; <v1.14: iris)")
parser.add_argument("--no-px4-autolaunch", action="store_true",
                    help="Do not start PX4; you start it yourself (see GUIDE.md section 4.3)")
parser.add_argument("--vehicle-id", type=int, default=0,
                    help="PX4 instance. TCP sim port = 4560+id, QGC still on UDP 14550, offboard 14540+id")
parser.add_argument("--spawn", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="Spawn position ENU [m]. Default: x=y=0 and z computed so the drone rests on the ground")
parser.add_argument("--rotor-constant", type=float, default=8.54858e-6,
                    help="Thrust curve T = k * w^2 [N/(rad/s)^2]. Raise it for a heavier custom airframe.")
parser.add_argument("--cam-yaw", type=float, default=0.0, help="Initial gimbal yaw in degrees (+ = left)")
parser.add_argument("--rolling-moment", type=float, default=1e-6,
                    help="Rotor drag torque coefficient (yaw moment = c * w^2)")
parser.add_argument("--gimbal-port", type=int, default=14600,
                    help="UDP port for gimbal commands: JSON {\"pitch\": deg down, \"yaw\": deg left} (0 = off)")
parser.add_argument("--max-rotor-vel", type=float, default=1100.0, help="Max rotor speed [rad/s]")
# Camera
parser.add_argument("--cam-mount", choices=["forward", "down"], default="forward")
parser.add_argument("--cam-offset", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="Camera position in the body frame (FLU, metres). Default 0.15,0,0.05 fwd / 0,0,-0.08 down")
parser.add_argument("--cam-tilt", type=float, default=0.0,
                    help="Extra pitch-down tilt of the forward camera in degrees (FPV style, e.g. 20)")
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--fps", type=float, default=30.0, help="Camera publish rate (render rate / integer divider)")
parser.add_argument("--hfov", type=float, default=90.0, help="Horizontal field of view in degrees")
parser.add_argument("--ros-namespace", default="drone", help="Topics: /<ns>/camera/image_raw, /<ns>/camera/camera_info")
parser.add_argument("--no-ros", action="store_true", help="Skip the ROS 2 camera publisher")
parser.add_argument("--rtsp-port", type=int, default=0,
                    help="Also stream the camera as H.264 RTSP on rtsp://localhost:<port>/drone_cam (0 = off, e.g. 8554)")
# App
parser.add_argument("--headless", action="store_true", help="No GUI (for CI/testing; QGC still works)")
parser.add_argument("--no-fast-forces", action="store_true",
                    help="Use Pegasus' stock force application (slower; for A/B comparison)")
parser.add_argument("--debug-state", action="store_true", help="Print attitude / body rates / acceleration every 2 s")
parser.add_argument("--max-seconds", type=float, default=0.0, help="Auto-exit after N wall-clock seconds (0 = never)")
args, _unknown = parser.parse_known_args()

# ----------------------------------------------------------------------------------------------------------------------
# 1. Start Isaac Sim. This MUST come before any omni.*, isaacsim.* (other than SimulationApp) or pxr import.
# ----------------------------------------------------------------------------------------------------------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "width": 1600,
    "height": 900,
    # Real-time path tracing is far too slow for a flying camera; RaytracedLighting is the interactive renderer.
    "renderer": "RaytracedLighting",
})

import carb
import numpy as np
import omni.graph.core as og
import omni.timeline
import omni.usd
import usdrt.Sdf
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics
from scipy.spatial.transform import Rotation

import isaacsim.core.experimental.utils.app as app_utils

# The ROS 2 bridge must be enabled before any of its OmniGraph node types are used.
if not args.no_ros:
    app_utils.enable_extension("isaacsim.ros2.bridge")
if args.rtsp_port:
    app_utils.enable_extension("isaacsim.streaming.rtsp")  # Isaac Sim 6 native RTSP server (kit-livestream)
simulation_app.update()

# Pegasus (installed in Isaac Sim's python with `pip install -e ~/PegasusSimulator/extensions/pegasus.simulator`)
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drone_usd_check import check_drone_usd, resting_height  # noqa: E402


class FastMultirotor(Multirotor):
    """Multirotor that caches the per-rotor RigidPrim handles.

    Pegasus 6.0's Vehicle.apply_force() builds a new isaacsim.core.experimental RigidPrim for every rotor on every
    physics step (4 x 250 Hz). Caching them is a pure performance change: forces are applied exactly the same way.
    """

    def start(self):
        self._rotor_prims = {}
        super().start()

    def stop(self):
        self._rotor_prims = {}
        super().stop()

    def apply_force(self, force, pos=[0.0, 0.0, 0.0], body_part="/body"):
        if body_part == "/body":
            return super().apply_force(force, pos, body_part)
        if self._body_rigid_prim is None or not self._body_rigid_prim.is_physics_tensor_entity_valid():
            return
        cache = self.__dict__.setdefault("_rotor_prims", {})
        prim = cache.get(body_part)
        if prim is None or not prim.is_physics_tensor_entity_valid():
            from isaacsim.core.experimental.prims import RigidPrim
            prim = cache[body_part] = RigidPrim(paths=self._stage_prefix + body_part)
        prim.apply_forces_and_torques_at_pos(
            forces=np.array([force], dtype=np.float32), torques=np.zeros((1, 3), dtype=np.float32),
            positions=np.array([pos], dtype=np.float32), local_frame=True)


ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
VEHICLE_PATH = "/World/drone"
CAMERA_NAME = "onboard_camera"
GIMBAL_PATH = f"{VEHICLE_PATH}/body/gimbal"
CAMERA_PATH = f"{GIMBAL_PATH}/{CAMERA_NAME}"
CAMERA_GRAPH_PATH = "/World/ROS2_CameraGraph"
GIMBAL_RATE = 60.0     # gimbal slew rate [deg/s]
RENDER_HZ = 60.0       # rendering_dt of Pegasus' px4 world settings
PHYSICS_HZ = 250.0     # physics_dt of Pegasus' px4 world settings (PX4 lockstep runs at this rate)


def log(msg):
    # carb.log_warn shows up in the Isaac Sim console without enabling verbose logging
    carb.log_warn(f"[launch_custom_drone] {msg}")
    print(f"[launch_custom_drone] {msg}", flush=True)


def resolve_drone_usd(usd):
    """Map shortcuts to the stock Pegasus assets and validate a custom USD."""
    if usd.lower() == "iris":
        return ROBOTS["Iris"]
    if usd.lower() == "pegasus":
        return ROBOTS["Pegasus"]
    usd = os.path.abspath(os.path.expanduser(usd))
    if not os.path.isfile(usd):
        log(f"ERROR: drone USD not found: {usd}\n"
            f"       Pass --usd /path/to/my_drone.usd (or --usd iris to test with the stock Iris).")
        simulation_app.close()
        sys.exit(1)
    errors, warnings, info = check_drone_usd(usd)
    for s in info:
        log(f"USD check: {s}")
    for s in warnings:
        log(f"USD check WARNING: {s}")
    if errors:
        for s in errors:
            log(f"USD check ERROR: {s}")
        log("The drone USD does not have the layout Pegasus needs (see GUIDE.md section 6.2). Aborting.")
        simulation_app.close()
        sys.exit(1)
    return usd


class DroneSim:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.stage = omni.usd.get_context().get_stage()
        self.stop_requested = False

        # --------------------------------------------------------------------------------------------------------------
        # 2. Physics world + environment
        # --------------------------------------------------------------------------------------------------------------
        self.pg = PegasusInterface()
        self.pg.set_world_settings(physics_dt=1.0 / PHYSICS_HZ, rendering_dt=1.0 / RENDER_HZ, stage_units_in_meters=1.0)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        self._build_environment()
        if args.objects:
            self._add_objects()
        # Apply the 250 Hz physics step AFTER the physics scene exists. If the scene is created afterwards it keeps
        # PhysX's 60 Hz default, the IMU/GPS stream PX4 gets is then inconsistent and PX4 refuses to arm
        # ("Attitude failure", "High Gyro Bias").
        self.pg.initialize_world()
        from isaacsim.core.simulation_manager import SimulationManager
        log(f"Physics dt = {SimulationManager.get_physics_dt():.4f} s ({1.0 / SimulationManager.get_physics_dt():.0f} Hz)")

        # --------------------------------------------------------------------------------------------------------------
        # 3. + 4. Drone and PX4 MAVLink backend
        # --------------------------------------------------------------------------------------------------------------
        usd_path = resolve_drone_usd(args.usd)
        px4_dir = os.path.abspath(os.path.expanduser(args.px4_dir))
        if not args.no_px4_autolaunch and not os.path.isfile(os.path.join(px4_dir, "build/px4_sitl_default/bin/px4")):
            log(f"ERROR: {px4_dir}/build/px4_sitl_default/bin/px4 not found. Build PX4 first: "
                f"cd {px4_dir} && make px4_sitl_default")
            simulation_app.close()
            sys.exit(1)

        # Spawn exactly at rest on the ground (see resting_height() for why a drop at start breaks PX4's EKF)
        if args.spawn is None:
            args.spawn = [0.0, 0.0, resting_height(usd_path)]
        log(f"Spawn position ENU = {np.round(args.spawn, 3).tolist()} m")

        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": args.vehicle_id,
            "connection_type": "tcpin",          # Isaac Sim is the TCP server, PX4's simulator_mavlink connects to it
            "connection_ip": "localhost",
            "connection_baseport": 4560,         # actual port = 4560 + vehicle_id
            "px4_autolaunch": not args.no_px4_autolaunch,
            "px4_dir": px4_dir,
            "px4_vehicle_model": args.px4_airframe,
            "enable_lockstep": True,             # PX4 waits for every sensor message: no drift when the sim slows down
            "update_rate": PHYSICS_HZ,
        })
        self.px4_backend = PX4MavlinkBackend(mavlink_config)
        if args.debug_state:
            _orig_send = self.px4_backend.send_sensor_msgs
            self._n_sent = 0
            def _logged_send(time_usec, _orig=_orig_send):
                if self._n_sent < 40:
                    d = self.px4_backend._sensor_data
                    log(f"DEBUG HIL#{self._n_sent} t={time_usec} acc=({d.xacc:.2f},{d.yacc:.2f},{d.zacc:.2f}) "
                        f"gyro=({d.xgyro:.3f},{d.ygyro:.3f},{d.zgyro:.3f})")
                self._n_sent += 1
                _orig(time_usec)
            self.px4_backend.send_sensor_msgs = _logged_send

        config = MultirotorConfig()
        config.thrust_curve = QuadraticThrustCurve({
            "num_rotors": 4,
            "rotor_constant": [args.rotor_constant] * 4,
            "rolling_moment_coefficient": [args.rolling_moment] * 4,
            "rot_dir": [-1, -1, 1, 1],           # PX4 quad-X: rotor0 FR (CCW), rotor1 RL (CCW), rotor2 FL (CW), rotor3 RR (CW)
            "min_rotor_velocity": [0.0] * 4,
            "max_rotor_velocity": [args.max_rotor_vel] * 4,
        })
        config.backends = [self.px4_backend]

        self.drone = (Multirotor if args.no_fast_forces else FastMultirotor)(
            VEHICLE_PATH,
            usd_path,
            args.vehicle_id,
            list(args.spawn),
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),  # [qx, qy, qz, qw]
            config=config,
        )
        log(f"Spawned drone from {usd_path} at {VEHICLE_PATH}")

        # --------------------------------------------------------------------------------------------------------------
        # 5. Onboard camera, a child of the body rigid body so it moves with the airframe
        # --------------------------------------------------------------------------------------------------------------
        self._create_camera()

        # --------------------------------------------------------------------------------------------------------------
        # 6. ROS 2 publishers (and optional RTSP stream)
        # --------------------------------------------------------------------------------------------------------------
        if not args.no_ros or args.rtsp_port:
            self._create_camera_graph()

        if not args.headless:
            self.pg.set_viewport_camera([-4.0, -4.0, 3.0], list(args.spawn))

    # ------------------------------------------------------------------------------------------------------------------
    def _physics_scene(self):
        # Physics scene with standard gravity (Pegasus' PX4 world settings expect 1 m units, Z up)
        if not any(p.IsA(UsdPhysics.Scene) for p in self.stage.Traverse()):
            scene = UsdPhysics.Scene.Define(self.stage, "/World/physicsScene")
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr(9.81)
            from pxr import PhysxSchema
            physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
            physx.CreateTimeStepsPerSecondAttr(int(PHYSICS_HZ))
            physx.CreateEnableGPUDynamicsAttr(False)   # one small articulation: CPU PhysX is faster and deterministic
            physx.CreateBroadphaseTypeAttr("MBP")

    def _lights(self, sun_intensity=3000.0, dome_intensity=800.0):
        sun = UsdLux.DistantLight.Define(self.stage, "/World/Lights/Sun")
        sun.CreateIntensityAttr(sun_intensity)
        sun.CreateAngleAttr(1.0)
        UsdGeom.XformCommonAPI(sun).SetRotate(Gf.Vec3f(-45.0, 0.0, 30.0))
        dome = UsdLux.DomeLight.Define(self.stage, "/World/Lights/Dome")
        dome.CreateIntensityAttr(dome_intensity)
        dome.CreateColorAttr(Gf.Vec3f(0.75, 0.85, 1.0))   # bluish sky fill

    def _add_reference(self, prim_path, usd_path, pos, yaw_deg=0.0, scale=1.0):
        prim = self.stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(usd_path)
        xf = UsdGeom.XformCommonAPI(prim)
        xf.SetTranslate(Gf.Vec3d(*pos))
        xf.SetRotate(Gf.Vec3f(0.0, 0.0, yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        xf.SetScale(Gf.Vec3f(scale, scale, scale))
        return prim

    def _add_objects(self):
        """Detection targets in front of the drone (it faces +X at spawn): a car and a person."""
        car = os.path.join(ASSETS_DIR, "car", "car.usd")
        person = os.path.join(ASSETS_DIR, "person", "person.usd")
        # Baylands is a gentle hill: z is the terrain height at that spot (sampled from the grass mesh)
        hill = args.env == "baylands"
        car_pos = (14.0, 4.0, 1.18 if hill else 0.0)
        person_pos = (11.0, -2.0, 0.78 if hill else 0.0)
        if os.path.isfile(car):
            self._add_reference("/World/Objects/car", car, car_pos, yaw_deg=25.0)
            log(f"Object: car at {car_pos} m")
        else:
            log(f"WARNING: {car} missing, no car in the scene")
        if os.path.isfile(person):
            self._add_reference("/World/Objects/person", person, person_pos, yaw_deg=200.0)
            log(f"Object: person at {person_pos} m")
        else:
            log(f"WARNING: {person} missing, no person in the scene")

    def _build_environment(self):
        if args.env == "baylands":
            self._physics_scene()
            usd = os.path.join(ASSETS_DIR, "baylands", "baylands.usd")
            if not os.path.isfile(usd):
                log(f"ERROR: {usd} not found (see tools/README in GUIDE.md); falling back to the ground plane")
                args.env = "ground"
            else:
                # Ground meshes (grass, paths, roads...) carry triangle-mesh colliders (tools/add_colliders.py); trees
                # and roofs do not. The park ground at the spawn point (origin) is at z = -0.04 m.
                self._add_reference("/World/Baylands", usd, (0.0, 0.0, 0.0))
                self._lights(3500.0, 900.0)
                log("Environment: Baylands park (Gazebo Fuel model converted to USD)")
        if args.env == "ground":
            self._physics_scene()
            from isaacsim.core.experimental.objects import GroundPlane
            GroundPlane("/World/GroundPlane", sizes=500.0)
            self._lights()
            # Static landmark boxes so the camera feed and the pilot have visual references
            landmarks = [((8, 0, 1), (1.0, 0.1, 0.1)), ((8, 4, 1), (0.1, 0.4, 1.0)), ((0, 8, 1), (0.1, 0.8, 0.2)),
                         ((-8, 0, 1), (1.0, 0.8, 0.1)), ((5, -6, 2), (0.8, 0.2, 0.8))]
            for i, (pos, color) in enumerate(landmarks):
                cube = UsdGeom.Cube.Define(self.stage, f"/World/Landmarks/box_{i}")
                cube.CreateSizeAttr(2.0 if pos[2] < 2 else 4.0)
                cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(*pos))
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())   # static collider (no RigidBodyAPI)
            log("Environment: local ground plane + landmarks")
        elif args.env != "baylands":
            path = SIMULATION_ENVIRONMENTS.get(args.env, args.env)
            log(f"Environment: loading {path}")
            # load_asset is synchronous (load_environment schedules an async task)
            self.pg.load_asset(path, "/World/layout")

    # ------------------------------------------------------------------------------------------------------------------
    def _create_camera(self):
        body_path = f"{VEHICLE_PATH}/body"
        if not self.stage.GetPrimAtPath(body_path).IsValid():
            log(f"ERROR: {body_path} does not exist after spawning; cannot attach the camera.")
            return

        cam = UsdGeom.Camera.Define(self.stage, CAMERA_PATH)

        # Optics: keep the default 20.955 mm horizontal aperture and derive focal length from the requested HFOV.
        #   hfov = 2 * atan(h_aperture / (2 * f))  ->  f = h_aperture / (2 * tan(hfov / 2))
        h_ap = 20.955
        focal = h_ap / (2.0 * math.tan(math.radians(args.hfov) / 2.0))
        v_ap = h_ap * args.height / args.width  # square pixels
        cam.CreateHorizontalApertureAttr(h_ap)
        cam.CreateVerticalApertureAttr(v_ap)
        cam.CreateFocalLengthAttr(focal)
        cam.CreateProjectionAttr("perspective")
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
        self.vfov = math.degrees(2.0 * math.atan(v_ap / (2.0 * focal)))

        # Gimbal: an Xform in the body frame (FLU: X forward, Y left, Z up) carrying the camera. Its rotation is
        # yaw about Z then pitch about Y (positive pitch = looking down), driven at run time over UDP (see run()).
        # A USD camera looks down its local -Z with +Y up, so rotateXYZ(90, 0, -90) points it along gimbal +X.
        if args.cam_offset:
            offset = args.cam_offset
        else:
            offset = [0.15, 0.0, 0.05] if args.cam_mount == "forward" else [0.0, 0.0, -0.08]
        self.gimbal_pitch = 90.0 if args.cam_mount == "down" else args.cam_tilt
        self.gimbal_yaw = args.cam_yaw
        self.gimbal_target = [self.gimbal_pitch, self.gimbal_yaw]
        gimbal = UsdGeom.Xform.Define(self.stage, GIMBAL_PATH)
        self.gimbal_xf = UsdGeom.XformCommonAPI(gimbal)
        self.gimbal_xf.SetTranslate(Gf.Vec3d(*offset))
        self._apply_gimbal()
        # small housing behind the lens so the gimbal is visible from outside
        housing = UsdGeom.Cube.Define(self.stage, f"{GIMBAL_PATH}/housing")
        housing.CreateSizeAttr(1.0)
        housing.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.05, 0.05)])
        hx = UsdGeom.XformCommonAPI(housing)
        hx.SetTranslate(Gf.Vec3d(-0.025, 0.0, 0.0))
        hx.SetScale(Gf.Vec3f(0.035, 0.035, 0.035))
        xf = UsdGeom.XformCommonAPI(cam)
        xf.SetRotate(Gf.Vec3f(90.0, 0.0, -90.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        rot = (self.gimbal_pitch, self.gimbal_yaw)
        log(f"Camera {CAMERA_PATH}: gimbal pitch {rot[0]:.0f} deg, yaw {rot[1]:.0f} deg, {args.width}x{args.height}, "
            f"HFOV {args.hfov:.1f} deg (VFOV {self.vfov:.1f}), focal {focal:.2f} mm, offset {offset}")

    def _apply_gimbal(self):
        self.gimbal_xf.SetRotate(Gf.Vec3f(0.0, self.gimbal_pitch, self.gimbal_yaw),
                                 UsdGeom.XformCommonAPI.RotationOrderXYZ)

    def _open_gimbal_socket(self):
        self.gimbal_sock = None
        if args.gimbal_port:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(("127.0.0.1", args.gimbal_port))
            except OSError as e:   # e.g. another sim still running: keep flying, just without gimbal control
                log(f"WARNING: gimbal port {args.gimbal_port} unavailable ({e}); gimbal control disabled")
                return
            sock.setblocking(False)
            self.gimbal_sock = sock
            log(f"Gimbal: UDP 127.0.0.1:{args.gimbal_port}, JSON {{'pitch': deg down, 'yaw': deg left}}")

    def _update_gimbal(self, dt):
        """Read the newest gimbal command and slew towards it (servo-like, GIMBAL_RATE deg/s wall clock)."""
        import json
        while self.gimbal_sock is not None:
            try:
                data = self.gimbal_sock.recv(512)
            except BlockingIOError:
                break
            try:
                cmd = json.loads(data.decode())
                self.gimbal_target = [max(-30.0, min(90.0, float(cmd.get("pitch", self.gimbal_target[0])))),
                                      max(-170.0, min(170.0, float(cmd.get("yaw", self.gimbal_target[1]))))]
            except (ValueError, AttributeError):
                pass
        step = GIMBAL_RATE * dt
        moved = False
        for i, attr in enumerate(("gimbal_pitch", "gimbal_yaw")):
            cur, tgt = getattr(self, attr), self.gimbal_target[i]
            if abs(tgt - cur) > 1e-3:
                setattr(self, attr, cur + max(-step, min(step, tgt - cur)))
                moved = True
        if moved:
            self._apply_gimbal()

    # ------------------------------------------------------------------------------------------------------------------
    def _create_camera_graph(self):
        """OmniGraph: OnTick -> IsaacCreateRenderProduct(camera) -> ROS2CameraHelper(rgb) + ROS2CameraInfoHelper,
        a /clock publisher so ROS nodes can use use_sim_time, and optionally an RTSPCameraHelper."""
        ns = args.ros_namespace.strip("/")
        keys = og.Controller.Keys
        nodes = [
            ("OnTick", "omni.graph.action.OnTick"),
            ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ]
        connect = [("OnTick.outputs:tick", "createRenderProduct.inputs:execIn")]
        values = [
            ("createRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_PATH)]),
            ("createRenderProduct.inputs:width", args.width),
            ("createRenderProduct.inputs:height", args.height),
        ]
        if not args.no_ros:
            nodes += [
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("publishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ]
            connect += [
                ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                ("OnTick.outputs:tick", "publishClock.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
            ]
            values += [
                # -> /drone/camera/image_raw  (sensor_msgs/Image, rgb8)
                ("cameraHelperRgb.inputs:nodeNamespace", ns),
                ("cameraHelperRgb.inputs:topicName", "camera/image_raw"),
                ("cameraHelperRgb.inputs:frameId", CAMERA_NAME),
                ("cameraHelperRgb.inputs:type", "rgb"),
                # -> /drone/camera/camera_info
                ("cameraHelperInfo.inputs:nodeNamespace", ns),
                ("cameraHelperInfo.inputs:topicName", "camera/camera_info"),
                ("cameraHelperInfo.inputs:frameId", CAMERA_NAME),
                ("publishClock.inputs:topicName", "clock"),
            ]
        if args.rtsp_port:
            nodes += [("rtspHelper", "isaacsim.streaming.rtsp.RTSPCameraHelper")]
            connect += [
                ("createRenderProduct.outputs:execOut", "rtspHelper.inputs:execIn"),
                ("createRenderProduct.outputs:renderProductPath", "rtspHelper.inputs:renderProductPath"),
            ]
            values += [("rtspHelper.inputs:port", args.rtsp_port), ("rtspHelper.inputs:mountPath", "/drone_cam")]

        graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": CAMERA_GRAPH_PATH,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {keys.CREATE_NODES: nodes, keys.CONNECT: connect, keys.SET_VALUES: values},
        )
        og.Controller.evaluate_sync(graph)
        simulation_app.update()

        # Throttle the RGB publisher: the sim renders at RENDER_HZ, publish every Nth frame.
        render_product_path = og.Controller.attribute(
            f"{CAMERA_GRAPH_PATH}/createRenderProduct.outputs:renderProductPath").get()
        import omni.syntheticdata
        import omni.syntheticdata._syntheticdata as sd
        step = max(1, int(round(RENDER_HZ / max(args.fps, 1e-3))))
        rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        try:
            gate = omni.syntheticdata.SyntheticData._get_node_path(rv_rgb + "IsaacSimulationGate", render_product_path)
            og.Controller.attribute(gate + ".inputs:step").set(step)
            info_gate = omni.syntheticdata.SyntheticData._get_node_path(
                "PostProcessDispatchIsaacSimulationGate", render_product_path)
            og.Controller.attribute(info_gate + ".inputs:step").set(step)
        except Exception as e:  # gate nodes are created lazily in some versions; publishing still works un-throttled
            log(f"WARNING: could not set camera publish rate ({e}); publishing every rendered frame")
        if not args.no_ros:
            log(f"ROS 2: /{ns}/camera/image_raw + /{ns}/camera/camera_info at ~{RENDER_HZ / step:.0f} Hz "
                f"(render {RENDER_HZ:.0f} Hz / {step}), /clock")
        if args.rtsp_port:
            log(f"RTSP: rtsp://localhost:{args.rtsp_port}/drone_cam")

    # ------------------------------------------------------------------------------------------------------------------
    def _kill_px4(self):
        tool = getattr(self.px4_backend, "px4_tool", None)
        if tool is not None:
            try:
                tool.kill_px4()
                log("PX4 SITL process stopped")
            except Exception as e:
                log(f"WARNING: could not stop PX4 cleanly: {e}")

    def request_stop(self, *_):
        self.stop_requested = True

    def run(self):
        # 7. Main loop. Pegasus registers physics callbacks on the SimulationManager, so simulation_app.update()
        #    advances physics (250 Hz substeps, PX4 lockstep), sensors, and one render frame per call.
        signal.signal(signal.SIGINT, self.request_stop)
        signal.signal(signal.SIGTERM, self.request_stop)

        self._open_gimbal_socket()
        self.timeline.play()
        log("Simulation playing. PX4 boots in a few seconds; open QGroundControl (UDP 14550).")
        t0 = time.time()
        last_status = t0
        try:
            last_t = time.time()
            while simulation_app.is_running() and not self.stop_requested:
                simulation_app.update()
                t_now = time.time()
                self._update_gimbal(t_now - last_t)
                last_t = t_now
                now = time.time()
                if args.max_seconds > 0 and now - t0 > args.max_seconds:
                    log(f"--max-seconds {args.max_seconds} reached")
                    break
                if args.debug_state and now - last_status > 2.0:
                    s = self.drone.state
                    eul = Rotation.from_quat(s.attitude).as_euler("XYZ", degrees=True)
                    log(f"DEBUG rpy={np.round(eul, 2)} w_body={np.round(s.angular_velocity, 4)} "
                        f"acc_world={np.round(s.linear_acceleration, 3)} v={np.round(s.linear_velocity, 4)}")
                    d = self.px4_backend._sensor_data
                    log(f"DEBUG HIL acc=({d.xacc:.3f},{d.yacc:.3f},{d.zacc:.3f}) gyro=({d.xgyro:.4f},{d.ygyro:.4f},"
                        f"{d.zgyro:.4f}) mag=({d.xmag:.3f},{d.ymag:.3f},{d.zmag:.3f}) p={d.abs_pressure:.2f} "
                        f"alt={d.pressure_alt:.2f} gps_alt={d.altitude} utime={self.px4_backend._current_utime}")
                if now - last_status > 10.0 or (args.debug_state and now - last_status > 2.0):
                    last_status = now
                    s = self.drone.state
                    sim_t = self.px4_backend._current_utime / 1e6
                    log(f"t={now - t0:5.0f}s  sim={sim_t:6.1f}s  pos ENU = [{s.position[0]:6.2f} {s.position[1]:6.2f} "
                        f"{s.position[2]:6.2f}] m")
        finally:
            log("Shutting down")
            self.timeline.stop()
            simulation_app.update()
            self._kill_px4()
            simulation_app.close()


def main():
    sim = DroneSim()
    sim.run()


if __name__ == "__main__":
    main()
