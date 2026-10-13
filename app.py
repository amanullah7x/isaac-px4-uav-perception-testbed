#!/usr/bin/env python3
"""
app.py - One-window launcher for the Isaac Sim + Pegasus + PX4 + QGroundControl drone pipeline.

    ./start_app.sh            (sources ROS 2 Humble and runs this file with ./venv/bin/python)

What the buttons do:
    Start simulation  -> ./run_sim.sh  (Isaac Sim + Pegasus quadcopter + PX4 SITL, scene with a car and a person,
                         onboard camera published on /drone/camera/image_raw)
    Open QGroundControl -> assets/QGroundControl.AppImage (take off, fly with the virtual joystick there)
    Stop all          -> stops Isaac Sim (which stops PX4) and QGroundControl

The camera feed from ROS 2 is shown in the window with YOLO detection boxes (models/best.pt, YOLOv8s VisDrone).
"""
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

# opencv may point Qt at its own plugin folder; PyQt5 must use its own.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

from PyQt5.QtCore import QProcess, Qt, QThread, QTimer, pyqtSignal  # noqa: E402
from PyQt5.QtGui import QFont, QImage, QPixmap, QTextCursor  # noqa: E402
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QFormLayout, QGroupBox, QHBoxLayout,  # noqa: E402
                             QLabel, QMainWindow, QPlainTextEdit, QPushButton, QSlider, QVBoxLayout, QWidget)

HERE = os.path.dirname(os.path.abspath(__file__))
RUN_SIM = os.path.join(HERE, "run_sim.sh")
GIMBAL_PORT = 14600
A = os.path.join(HERE, "assets")
# name -> launch_custom_drone.py arguments. PX4 airframes isaac_x500 / isaac_fpv are in
# ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes (2200x_isaac_*).
DRONES = {
    "Pegasus quadcopter (1.5 kg)": ["--usd", f"{A}/pegasus.usd", "--px4-airframe", "isaac_iris"],
    "Holybro X500 (PX4 model, 2.1 kg)": ["--usd", f"{A}/x500/x500.usd", "--px4-airframe", "isaac_x500",
                                         "--rotor-constant", "8.54858e-6", "--max-rotor-vel", "1100",
                                         "--cam-offset", "0.22", "0", "-0.05"],
    "FPV 5-inch racer (0.6 kg)": ["--usd", f"{A}/fpv/fpv.usd", "--px4-airframe", "isaac_fpv",
                                  "--rotor-constant", "5.6e-6", "--max-rotor-vel", "1100",
                                  "--rolling-moment", "6.5e-7", "--cam-offset", "0.075", "0", "0.03"],
    "3DR Iris (Pegasus stock)": ["--usd", "iris", "--px4-airframe", "isaac_iris"],
}
QGC = os.path.join(HERE, "assets", "QGroundControl.AppImage")
MODEL = os.path.join(HERE, "models", "best.pt")
TOPIC = "/drone/camera/image_raw"

# VisDrone classes in best.pt; the ones the scene contains get bright colours.
CLASS_COLORS = {"pedestrian": (0, 255, 0), "people": (0, 200, 0), "car": (255, 140, 0), "van": (255, 200, 0),
                "truck": (255, 80, 0), "bus": (255, 0, 120)}


def msg_to_rgb(msg):
    ch = 4 if msg.encoding in ("rgba8", "bgra8") else 3
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * ch]
    img = img.reshape(msg.height, msg.width, ch)
    if msg.encoding == "rgba8":
        img = img[:, :, :3]
    elif msg.encoding in ("bgr8", "bgra8"):
        img = cv2.cvtColor(img[:, :, :3], cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(img)


class CameraThread(QThread):
    """Subscribes to the drone camera, runs YOLO on the newest frame, emits an annotated RGB image."""
    frame = pyqtSignal(QImage, str)

    def __init__(self):
        super().__init__()
        self.detect = True
        self.conf = 0.25
        self._latest = None
        self._lock = threading.Lock()
        self._running = True
        self.model = None
        self.model_error = ""

    def _on_image(self, msg):
        with self._lock:
            self._latest = msg

    def stop(self):
        self._running = False

    def run(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        try:
            from ultralytics import YOLO
            self.model = YOLO(MODEL)
            self.model.predict(np.zeros((480, 640, 3), np.uint8), verbose=False)  # warm-up (loads CUDA kernels)
        except Exception as e:  # keep showing the raw feed even if the model fails to load
            self.model_error = str(e)

        rclpy.init()
        node = rclpy.create_node("drone_app_viewer")
        node.create_subscription(Image, TOPIC, self._on_image, qos_profile_sensor_data)
        spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spinner.start()

        n, t0, fps = 0, time.time(), 0.0
        while self._running:
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is None:
                time.sleep(0.005)
                continue
            img = msg_to_rgb(msg)
            info = []
            if self.detect and self.model is not None:
                res = self.model.predict(img[:, :, ::-1], conf=self.conf, verbose=False)[0]
                counts = {}
                for box, cls, cf in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(),
                                        res.boxes.conf.cpu().numpy()):
                    name = res.names[int(cls)]
                    counts[name] = counts.get(name, 0) + 1
                    color = CLASS_COLORS.get(name, (0, 200, 255))
                    x1, y1, x2, y2 = box.astype(int)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                    label = f"{name} {cf:.2f}"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(img, (x1, max(0, y1 - th - 6)), (x1 + tw + 4, y1), color, -1)
                    cv2.putText(img, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                info.append(", ".join(f"{k}: {v}" for k, v in counts.items()) or "no detections")
            elif self.model is None:
                info.append(f"detector not loaded: {self.model_error[:80]}")
            n += 1
            if time.time() - t0 >= 1.0:
                fps, n, t0 = n / (time.time() - t0), 0, time.time()
            info.insert(0, f"{msg.width}x{msg.height}  {fps:4.1f} FPS")
            h, w, _ = img.shape
            qimg = QImage(img.data, w, h, 3 * w, QImage.Format_RGB888).copy()
            self.frame.emit(qimg, "   |   ".join(info))

        node.destroy_node()
        rclpy.shutdown()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Sim + Detection")
        self.setFocusPolicy(Qt.StrongFocus)
        self.sim = None
        self.qgc = None
        self.last_frame_t = 0.0

        # Video
        self.video = QLabel("Camera feed appears here once the simulation is running")
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(640, 480)
        self.video.setStyleSheet("background:#111; color:#aaa; font-size:14px;")
        self.video_info = QLabel("")
        self.video_info.setStyleSheet("font-family:monospace;")

        # Controls
        self.scene = QComboBox()
        self.scene.addItems(["Baylands (outdoor park)", "Simple ground + landmarks"])
        self.drone = QComboBox()
        self.drone.addItems(list(DRONES))

        # Gimbal (works before and during flight; also keys W/S = pitch, A/D = yaw, C = centre)
        self.gimbal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.g_pitch = QSlider(Qt.Horizontal)
        self.g_pitch.setRange(-30, 90)
        self.g_pitch.setValue(35)
        self.g_yaw = QSlider(Qt.Horizontal)
        self.g_yaw.setRange(-170, 170)
        self.g_yaw.setValue(0)
        self.g_pitch_lbl, self.g_yaw_lbl = QLabel("35° down"), QLabel("0°")
        for sl in (self.g_pitch, self.g_yaw):
            sl.valueChanged.connect(self.send_gimbal)
        presets = QHBoxLayout()
        for label, pitch in (("Level", 0), ("45°", 45), ("Down", 90)):
            b = QPushButton(label)
            b.clicked.connect(lambda _c, v=pitch: self.g_pitch.setValue(v))
            presets.addWidget(b)
        b = QPushButton("Centre")
        b.clicked.connect(lambda: self.g_yaw.setValue(0))
        presets.addWidget(b)
        from PyQt5.QtWidgets import QShortcut
        from PyQt5.QtGui import QKeySequence
        for key, sl, d in (("W", "p", -5), ("S", "p", 5), ("A", "y", 5), ("D", "y", -5)):
            sc = QShortcut(QKeySequence(key), self)
            sc.activated.connect(lambda s_=sl, d_=d: (self.g_pitch if s_ == "p" else self.g_yaw).setValue(
                (self.g_pitch if s_ == "p" else self.g_yaw).value() + d_))
        QShortcut(QKeySequence("C"), self).activated.connect(lambda: self.g_yaw.setValue(0))
        self.headless = QCheckBox("Hide Isaac Sim window (faster)")
        self.btn_start = QPushButton("Start simulation")
        self.btn_qgc = QPushButton("Open QGroundControl")
        self.btn_stop = QPushButton("Stop all")
        self.btn_start.clicked.connect(self.start_sim)
        self.btn_qgc.clicked.connect(self.start_qgc)
        self.btn_stop.clicked.connect(self.stop_all)

        self.detect = QCheckBox("Detection enabled")
        self.detect.setChecked(True)
        self.conf = QSlider(Qt.Horizontal)
        self.conf.setRange(5, 90)
        self.conf.setValue(25)
        self.conf_lbl = QLabel("0.25")

        self.st_sim, self.st_px4, self.st_qgc, self.st_cam = (QLabel("-") for _ in range(4))

        sim_box = QGroupBox("Simulation")
        f = QFormLayout(sim_box)
        f.addRow("Scene", self.scene)
        f.addRow("Drone", self.drone)
        f.addRow(self.headless)
        f.addRow(self.btn_start)
        f.addRow(self.btn_qgc)
        f.addRow(self.btn_stop)

        gim_box = QGroupBox("Camera gimbal (keys W/S/A/D, C)")
        fg = QFormLayout(gim_box)
        r1 = QHBoxLayout(); r1.addWidget(self.g_pitch); r1.addWidget(self.g_pitch_lbl)
        r2 = QHBoxLayout(); r2.addWidget(self.g_yaw); r2.addWidget(self.g_yaw_lbl)
        fg.addRow("Pitch", r1)
        fg.addRow("Yaw", r2)
        fg.addRow(presets)

        det_box = QGroupBox("Detection (YOLOv8s VisDrone)")
        f2 = QFormLayout(det_box)
        f2.addRow(self.detect)
        row = QHBoxLayout()
        row.addWidget(self.conf)
        row.addWidget(self.conf_lbl)
        f2.addRow("Confidence", row)

        st_box = QGroupBox("Status")
        f3 = QFormLayout(st_box)
        f3.addRow("Isaac Sim", self.st_sim)
        f3.addRow("PX4 SITL", self.st_px4)
        f3.addRow("QGroundControl", self.st_qgc)
        f3.addRow("Camera", self.st_cam)

        help_lbl = QLabel("In QGroundControl: wait for 'Ready To Fly', click Takeoff, then use the virtual "
                          "joystick (Application Settings > Fly View > Virtual Joystick) to move the drone.")
        help_lbl.setWordWrap(True)

        side = QVBoxLayout()
        for w in (sim_box, gim_box, det_box, st_box, help_lbl):
            side.addWidget(w)
        side.addStretch(1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        self.log.setFont(QFont("Monospace", 8))
        self.log.setFixedHeight(160)

        left = QVBoxLayout()
        left.addWidget(self.video, 1)
        left.addWidget(self.video_info)
        left.addWidget(self.log)
        top = QHBoxLayout()
        top.addLayout(left, 1)
        side_w = QWidget()
        side_w.setLayout(side)
        side_w.setFixedWidth(350)
        top.addWidget(side_w)
        root = QWidget()
        root.setLayout(top)
        self.setCentralWidget(root)
        self.resize(1150, 880)

        # Camera / detector thread
        self.cam_thread = CameraThread()
        self.cam_thread.frame.connect(self.on_frame)
        self.detect.toggled.connect(lambda v: setattr(self.cam_thread, "detect", v))
        self.conf.valueChanged.connect(self.on_conf)
        self.cam_thread.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(1000)
        self.refresh_status()

    # ------------------------------------------------------------------------------------------------------------------
    def append_log(self, text):
        self.log.moveCursor(QTextCursor.End)
        self.log.insertPlainText(text)
        self.log.moveCursor(QTextCursor.End)

    def send_gimbal(self, *_):
        p, y = self.g_pitch.value(), self.g_yaw.value()
        self.g_pitch_lbl.setText(f"{p}° {'down' if p >= 0 else 'up'}")
        self.g_yaw_lbl.setText(f"{abs(y)}° {'left' if y > 0 else 'right' if y < 0 else ''}")
        try:
            self.gimbal_sock.sendto(json.dumps({"pitch": p, "yaw": y}).encode(), ("127.0.0.1", GIMBAL_PORT))
        except OSError:
            pass

    def on_conf(self, v):
        self.cam_thread.conf = v / 100.0
        self.conf_lbl.setText(f"{v / 100:.2f}")

    def on_frame(self, qimg, info):
        self.last_frame_t = time.time()
        pix = QPixmap.fromImage(qimg).scaled(self.video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video.setPixmap(pix)
        self.video_info.setText(info)

    def sim_args(self):
        a = DRONES[self.drone.currentText()] + [
            "--env", "baylands" if self.scene.currentIndex() == 0 else "ground", "--objects",
            "--cam-tilt", str(self.g_pitch.value()), "--gimbal-port", str(GIMBAL_PORT)]
        if self.headless.isChecked():
            a.append("--headless")
        return a

    def start_sim(self):
        if self.sim is not None and self.sim.state() != QProcess.NotRunning:
            self.append_log("[app] simulation is already running\n")
            return
        if subprocess.run(["pgrep", "-x", "px4"], capture_output=True).returncode == 0:
            self.append_log("[app] stopping a PX4 left over from an earlier run\n")
            subprocess.run(["pkill", "-x", "px4"])
            time.sleep(1.0)
        self.sim = QProcess(self)
        self.sim.setProcessChannelMode(QProcess.MergedChannels)
        self.sim.setWorkingDirectory(HERE)
        self.sim.readyReadStandardOutput.connect(
            lambda: self.append_log(bytes(self.sim.readAllStandardOutput()).decode(errors="replace")))
        self.sim.finished.connect(lambda code, _s: self.append_log(f"[app] Isaac Sim exited ({code})\n"))
        args = self.sim_args()
        self.append_log(f"[app] starting: run_sim.sh {' '.join(args)}\n"
                        "[app] Isaac Sim takes about a minute to load the scene.\n")
        # setsid: the sim gets its own process group, so Stop reaches Isaac Sim's python behind python.sh too
        self.sim.start("setsid", [RUN_SIM] + args)

    def start_qgc(self):
        if self.qgc is not None and self.qgc.state() != QProcess.NotRunning:
            self.append_log("[app] QGroundControl is already open\n")
            return
        self.qgc = QProcess(self)
        self.qgc.setProcessChannelMode(QProcess.MergedChannels)
        self.qgc.readyReadStandardOutput.connect(lambda: self.qgc.readAllStandardOutput())  # discard, it is noisy
        self.append_log("[app] opening QGroundControl\n")
        self.qgc.start(QGC, [])

    def stop_all(self):
        if self.sim is not None and self.sim.state() != QProcess.NotRunning:
            self.append_log("[app] stopping Isaac Sim and PX4...\n")
            pgid = int(self.sim.processId())
            try:
                os.killpg(pgid, signal.SIGTERM)          # launch_custom_drone.py shuts PX4 down on SIGTERM
                if not self.sim.waitForFinished(20000):
                    os.killpg(pgid, signal.SIGKILL)
                    self.sim.waitForFinished(3000)
            except ProcessLookupError:
                pass
            time.sleep(1.0)
            try:
                os.killpg(pgid, signal.SIGKILL)          # anything left in the group (python.sh's child)
            except ProcessLookupError:
                pass
        subprocess.run(["pkill", "-x", "px4"])
        if self.qgc is not None and self.qgc.state() != QProcess.NotRunning:
            self.qgc.terminate()
            if not self.qgc.waitForFinished(5000):
                self.qgc.kill()
        self.append_log("[app] stopped\n")

    def refresh_status(self):
        if self.sim is not None and self.sim.state() != QProcess.NotRunning:
            self.send_gimbal()
        def mark(lbl, ok, on="running", off="stopped"):
            lbl.setText(on if ok else off)
            lbl.setStyleSheet("color:#2a2;" if ok else "color:#888;")
        mark(self.st_sim, self.sim is not None and self.sim.state() != QProcess.NotRunning)
        mark(self.st_px4, subprocess.run(["pgrep", "-x", "px4"], capture_output=True).returncode == 0)
        mark(self.st_qgc, self.qgc is not None and self.qgc.state() != QProcess.NotRunning, "open", "closed")
        mark(self.st_cam, time.time() - self.last_frame_t < 2.0, "receiving", "no frames")

    def closeEvent(self, ev):
        self.stop_all()
        self.cam_thread.stop()
        self.cam_thread.wait(3000)
        ev.accept()


def main():
    if not os.environ.get("AMENT_PREFIX_PATH"):
        sys.exit("ROS 2 is not sourced. Start the app with ./start_app.sh")
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
