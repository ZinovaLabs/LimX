"""
Robot-state and camera sources for Tron2 VLA data collection.

State / action layout (same as tron2_env's default PhysicalLayout and tron2_openpi's
16-dim TRON2 policy, see vla/README.md):

    [left_arm_0..6, left_gripper, right_arm_0..6, right_gripper]

Arm values are joint positions in radians, in the robot's own motor order
(motor_0..6 left arm, motor_7..13 right arm: proximal pitch, roll, yaw, elbow, wrist yaw,
wrist pitch, wrist roll). Gripper values are the opening normalised to 0..1
(tron2_env multiplies by 100 for the robot's percentage command).

Camera sources, all delivering JPEG bytes per camera name (cam_high, cam_left_wrist,
cam_right_wrist, as in tron2_openpi):

    sdk        the robot's compressed image topics read through the LimX SDK
               (no extra dependency): head camera /camera/top/color/image_raw/compressed,
               wrist cameras /camera/left|right/color/image_rect_raw/compressed
    bridge     tron2_env's BridgeObservationProvider (TRON2 Bridge WebSocket)
               needs: pip install -e "tron2_env[bridge]" and the Bridge host
    realsense  tron2_env's MultiCameraManager for RealSense cameras attached to this PC
               needs: pip install -e "tron2_env[camera]" and the camera serials

Everything here only reads from the robot.
"""

import io
import struct
import threading
import time

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
from limxsdk.msg import Header

ARM_JOINTS = ["proximal_pitch", "proximal_roll", "proximal_yaw", "elbow",
              "wrist_yaw", "wrist_pitch", "wrist_roll"]
LAYOUT_NAMES = ([f"left_arm_{i}" for i in range(7)] + ["left_gripper"]
                + [f"right_arm_{i}" for i in range(7)] + ["right_gripper"])
LEFT_ARM_MOTORS = list(range(0, 7))
RIGHT_ARM_MOTORS = list(range(7, 14))
CAMERA_NAMES = ("cam_high", "cam_left_wrist", "cam_right_wrist")

# Robot camera topics: the head camera and one camera per arm end-effector. The wrist
# cameras are advertised on the robot as image_rect_raw; tron2_env's Bridge defaults
# (image_resized) and the raw variant are tried as fallbacks.
# The first topic of a camera that delivers an image is used.
SDK_CAMERA_TOPICS = {
    "cam_high": ["/camera/top/color/image_raw/compressed"],
    "cam_left_wrist": ["/camera/left/color/image_rect_raw/compressed",
                       "/camera/left/color/image_resized/compressed",
                       "/camera/left/color/image_raw/compressed"],
    "cam_right_wrist": ["/camera/right/color/image_rect_raw/compressed",
                        "/camera/right/color/image_resized/compressed",
                        "/camera/right/color/image_raw/compressed"],
}

_u32 = struct.Struct("<I").unpack_from


def build_vector(motor_q, gripper_open):
    """16-dim [left arm 7, left gripper, right arm 7, right gripper] from the 16 motor
    positions and a (left, right) gripper opening in 0..1."""
    return ([float(motor_q[i]) for i in LEFT_ARM_MOTORS] + [float(gripper_open[0])]
            + [float(motor_q[i]) for i in RIGHT_ARM_MOTORS] + [float(gripper_open[1])])


# ----------------------------------------------------------------------------- SDK messages

class CompressedImage(object):
    """Mirror of `sensor_msgs/CompressedImage` for the SDK's generic subscriber (decode only)."""

    TYPE = "sensor_msgs/CompressedImage"
    MD5 = "8f7a12909da2c9d3332d540a0977563f"
    MIN_ENCODED_SIZE = 24
    __slots__ = ("header", "format", "data")

    @staticmethod
    def definition():
        return ("std_msgs/Header header\nstring format\nuint8[] data\n"
                "================================================================================\n"
                "MSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n")

    @classmethod
    def decode(cls, buf, offset=0):
        self = cls.__new__(cls)
        self.header, o = Header.decode(buf, offset)
        n = _u32(buf, o)[0]
        o += 4
        if o + n > len(buf):
            raise ValueError("CompressedImage.format overruns the frame")
        self.format = str(buf[o:o + n], "utf-8", "replace")
        o += n
        n = _u32(buf, o)[0]
        o += 4
        if o + n > len(buf):
            raise ValueError("CompressedImage.data overruns the frame")
        self.data = bytes(buf[o:o + n])
        return self, o + n


# ----------------------------------------------------------------------------- robot state

class RobotSource(object):
    """Latest joint state, robot-controller command and gripper state, with receive times."""

    def __init__(self, ip):
        self.robot = Robot(RobotType.Tron2)
        if not self.robot.init(ip):
            raise RuntimeError("robot.init failed for {}".format(ip))
        self.lock = threading.Lock()
        self.state = None        # (t, q, dq, tau)
        self.cmd = None          # (t, q, Kp) from the robot's own controller (/motor/cmd)
        self.gripper = None      # (t, [left, right] opening in percent)
        self.robot.subscribeRobotState(self._on_state)
        self.robot.subscribeRobotCmd(self._on_cmd)
        self.robot.subscribeGripperState(self._on_gripper)

    def _on_state(self, s):
        with self.lock:
            self.state = (time.monotonic(), list(s.q), list(s.dq), list(s.tau))

    def _on_cmd(self, c):
        with self.lock:
            self.cmd = (time.monotonic(), list(c.q), list(c.Kp))

    def _on_gripper(self, g):
        with self.lock:
            self.gripper = (time.monotonic(), list(g.q))

    def snapshot(self):
        with self.lock:
            return self.state, self.cmd, self.gripper


# ----------------------------------------------------------------------------- cameras

class CameraSource(object):
    """Common interface: latest(name) -> (jpeg_bytes, receive_time, seq) or None."""

    names = ()

    def latest(self, name):
        raise NotImplementedError

    def close(self):
        pass


class SdkCameraSource(CameraSource):
    """Compressed image topics read with the LimX SDK's generic subscriber.

    topics: {name: topic or [candidate topics]}; the first candidate that delivers an
    image becomes the camera's topic, the others are ignored from then on."""

    def __init__(self, robot, topics):
        self.robot = robot
        self.lock = threading.Lock()
        self.frames = {}
        self.topics = {n: [t] if isinstance(t, str) else list(t) for n, t in topics.items()}
        self.active = {}
        self.names = tuple(self.topics)
        self.subs = []
        for name, candidates in self.topics.items():
            for topic in candidates:
                def cb(msg, name=name, topic=topic):
                    with self.lock:
                        if self.active.setdefault(name, topic) != topic:
                            return
                        seq = self.frames.get(name, (None, 0, 0))[2] + 1
                        self.frames[name] = (msg.data, time.monotonic(), seq)
                self.subs.append(robot.subscribe(CompressedImage, topic, cb))

    def latest(self, name):
        with self.lock:
            return self.frames.get(name)

    def active_topic(self, name):
        with self.lock:
            return self.active.get(name)

    def published_topics(self):
        """Topic names that currently have a publisher on the robot (takes up to 3 s)."""
        return {t["name"] for t in self.robot.get_published_topics()}

    def close(self):
        for s in self.subs:
            s.close()


def _encode_jpeg(rgb, quality=90):
    from PIL import Image  # tron2_env depends on Pillow
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class BridgeCameraSource(CameraSource):
    """tron2_env's BridgeObservationProvider (images from the TRON2 Bridge WebSocket).

    The provider returns OpenPI-convention RGB images keyed cam_high / cam_left_wrist /
    cam_right_wrist; they are re-encoded to JPEG here."""

    def __init__(self, host, ws_path="/bridge/ws", image_max_fps=0, verify_tls=False):
        from tron2_env.bridge import BridgeConfig, BridgeObservationProvider
        self.provider = BridgeObservationProvider(BridgeConfig(
            host=host, ws_path=ws_path, image_max_fps=image_max_fps, verify_tls=verify_tls,
            save_debug_images=False))
        self.names = CAMERA_NAMES
        self.lock = threading.Lock()
        self.frames = {}
        self.running = True
        self.provider.start()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                obs = self.provider.get_obs(timeout=1.0)
            except TimeoutError:
                continue
            now = time.monotonic()
            for name, img in (obs.get("images") or {}).items():
                if img is None:
                    continue
                if img.ndim == 3 and img.shape[0] == 3 and img.shape[-1] != 3:
                    img = img.transpose(1, 2, 0)          # CHW -> HWC
                data = _encode_jpeg(img)
                with self.lock:
                    seq = self.frames.get(name, (None, 0, 0))[2] + 1
                    self.frames[name] = (data, now, seq)

    def latest(self, name):
        with self.lock:
            return self.frames.get(name)

    def close(self):
        self.running = False
        self.provider.stop()


class RealSenseCameraSource(CameraSource):
    """tron2_env's MultiCameraManager for RealSense cameras attached to this PC.

    serial_to_name: {"<serial>": "cam_high", ...}. RealSense delivers BGR; converted to RGB."""

    def __init__(self, serial_to_name, width=640, height=480, fps=30):
        from tron2_env.camera import MultiCameraManager
        configs = {name: {"color_width": width, "color_height": height, "fps": fps}
                   for name in serial_to_name.values()}
        self.manager = MultiCameraManager(serial_to_name=serial_to_name, camera_configs=configs)
        self.manager.setup_pipelines()
        self.manager.start_capture()
        self.names = tuple(serial_to_name.values())
        self.lock = threading.Lock()
        self.frames = {}
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            got = False
            for name, frame in self.manager.get_all_latest_frames().items():
                if not frame or frame.get("color") is None:
                    continue
                got = True
                data = _encode_jpeg(frame["color"][:, :, ::-1].copy())   # BGR -> RGB
                with self.lock:
                    seq = self.frames.get(name, (None, 0, 0))[2] + 1
                    self.frames[name] = (data, time.monotonic(), seq)
            if not got:
                time.sleep(0.005)

    def latest(self, name):
        with self.lock:
            return self.frames.get(name)

    def close(self):
        self.running = False
        self.manager.stop_capture()
