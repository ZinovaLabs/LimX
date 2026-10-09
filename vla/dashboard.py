"""
Live Tron2 dashboard: the three cameras, both end-effectors' position and the end-effector
contact force, in one window. Read only, so it runs on its own or next to collect_vla_data.py.

    source ~/limx-venv/bin/activate
    python3 vla/dashboard.py                 # --ip 10.192.1.2 --window 10

Panels:
    cameras     cam_high, cam_left_wrist, cam_right_wrist with rate and image age
                (red when an image is older than 0.5 s)
    per arm     position X/Y/Z of the gripper mount (mm, robot base frame, computed from the
                joint angles with the URDF) as large readouts, the change of X/Y/Z since
                the reference (Δ mm) as curves, and the contact force Fx/Fy/Fz and |F| (N)
                as curves

The force is the robot's own estimate (/dyn_identify/ee_force_kf, no force sensor); it
reads ~20 N at rest, so press "Tare" with the arms free to show contact force only.

Keys: space pause / resume · r reset the Δ reference · t tare the force · 1/2/3 show
5/10/30 s · q or Esc quit.

Needs: pip install pyqtgraph PyQt6
"""

import argparse
import collections
import os
import sys
import time

import numpy as np

try:
    import pyqtgraph as pg
    from PyQt6 import QtCore, QtGui, QtWidgets
except ImportError:
    sys.exit("The dashboard needs pyqtgraph and PyQt6: pip install pyqtgraph PyQt6")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tron2_vla_io import (CAMERA_NAMES, SDK_CAMERA_TOPICS, ArmKinematics,  # noqa: E402
                          ArmSource, RobotSource, SdkCameraSource)

# ----------------------------------------------------------------------------- look

BG = "#0d1017"
CARD = "#151a24"
CARD_EDGE = "#232a38"
TEXT = "#e7eaf0"
MUTED = "#8a93a6"
ACCENT = "#7c6cff"
OK = "#34d399"
WARN = "#fbbf24"
BAD = "#f87171"
AXIS_COLORS = {"X": "#ff6b6b", "Y": "#4ade80", "Z": "#60a5fa"}   # same in every plot
MAG_COLOR = "#e7eaf0"

STALE_S = 0.5            # camera image / stream older than this: shown as stale
WINDOWS = (5, 10, 30)    # selectable plot spans, seconds
PLOT_RATE = 200          # samples per second kept for the curves (the pose arrives at ~900)
MIN_SPAN = {"pos": 5.0, "force": 5.0}   # y axis shows at least +-this (mm, N): noise stays flat

STYLE = """
QMainWindow, QWidget#root {{ background: {bg}; }}
QLabel {{ color: {text}; }}
QFrame#card {{ background: {card}; border: 1px solid {edge}; border-radius: 14px; }}
QLabel#title {{ font-size: 20px; font-weight: 700; letter-spacing: 0.5px; }}
QLabel#subtitle {{ color: {muted}; font-size: 12px; }}
QLabel#cardTitle {{ font-size: 13px; font-weight: 700; letter-spacing: 1.5px; color: {text}; }}
QLabel#cardHint {{ color: {muted}; font-size: 11px; }}
QLabel#chip {{ border-radius: 10px; padding: 3px 10px; font-size: 11px; font-weight: 600; }}
QLabel#axisName {{ font-size: 11px; font-weight: 700; }}
QLabel#axisValue {{ font-size: 22px; font-weight: 600; font-family: "DejaVu Sans Mono", monospace; }}
QLabel#axisUnit {{ color: {muted}; font-size: 11px; }}
QPushButton {{ background: {card}; color: {text}; border: 1px solid {edge}; border-radius: 9px;
              padding: 6px 14px; font-size: 12px; font-weight: 600; }}
QPushButton:hover {{ border-color: {accent}; }}
QPushButton:checked {{ background: {accent}; border-color: {accent}; }}
""".format(bg=BG, card=CARD, edge=CARD_EDGE, text=TEXT, muted=MUTED, accent=ACCENT)


def chip(text, color):
    """Small rounded status label."""
    label = QtWidgets.QLabel(text)
    label.setObjectName("chip")
    set_chip(label, text, color)
    return label


def set_chip(label, text, color):
    label.setText(text)
    c = QtGui.QColor(color)
    label.setStyleSheet("background: rgba({},{},{},40); color: {}; border: 1px solid rgba({},{},{},110);"
                        .format(c.red(), c.green(), c.blue(), color, c.red(), c.green(), c.blue()))


def card(title, hint=""):
    """Rounded panel with a title row; returns (frame, body layout)."""
    frame = QtWidgets.QFrame()
    frame.setObjectName("card")
    outer = QtWidgets.QVBoxLayout(frame)
    outer.setContentsMargins(14, 12, 14, 12)
    outer.setSpacing(8)
    head = QtWidgets.QHBoxLayout()
    t = QtWidgets.QLabel(title.upper())
    t.setObjectName("cardTitle")
    head.addWidget(t)
    head.addStretch(1)
    if hint:
        h = QtWidgets.QLabel(hint)
        h.setObjectName("cardHint")
        head.addWidget(h)
    outer.addLayout(head)
    return frame, outer


# ----------------------------------------------------------------------------- widgets

class CameraView(QtWidgets.QWidget):
    """One camera: the image scaled to fit with rounded corners, name and status on top."""

    def __init__(self, name):
        super().__init__()
        self.name = name.replace("cam_", "").replace("_", " ").upper()
        self.pixmap = None
        self.status, self.status_color = "waiting", MUTED
        self.setMinimumSize(240, 150)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)

    def set_jpeg(self, data):
        pixmap = QtGui.QPixmap()
        if pixmap.loadFromData(data):
            self.pixmap = pixmap
            self.update()

    def set_status(self, text, color):
        if (text, color) != (self.status, self.status_color):
            self.status, self.status_color = text, color
            self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
        r = QtCore.QRectF(self.rect())
        path = QtGui.QPainterPath()
        path.addRoundedRect(r, 12, 12)
        p.fillPath(path, QtGui.QColor("#0a0d13"))
        if self.pixmap is not None:
            size = self.pixmap.size().scaled(self.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
            target = QtCore.QRectF((r.width() - size.width()) / 2, (r.height() - size.height()) / 2,
                                   size.width(), size.height())
            p.setClipPath(path)
            p.drawPixmap(target, self.pixmap, QtCore.QRectF(self.pixmap.rect()))
            p.setClipping(False)
        else:
            p.setPen(QtGui.QColor(MUTED))
            p.drawText(r, QtCore.Qt.AlignmentFlag.AlignCenter, "no image yet")

        # top gradient so the labels read on any image
        grad = QtGui.QLinearGradient(0, 0, 0, 44)
        grad.setColorAt(0, QtGui.QColor(0, 0, 0, 170))
        grad.setColorAt(1, QtGui.QColor(0, 0, 0, 0))
        p.setClipPath(path)
        p.fillRect(QtCore.QRectF(0, 0, r.width(), 44), grad)
        p.setClipping(False)

        font = p.font()
        font.setBold(True)
        font.setPixelSize(12)
        font.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 1.2)
        p.setFont(font)
        p.setPen(QtGui.QColor(TEXT))
        p.drawText(QtCore.QRectF(12, 8, r.width() - 24, 20), QtCore.Qt.AlignmentFlag.AlignLeft, self.name)

        # status pill, top right
        font.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 0)
        font.setPixelSize(11)
        p.setFont(font)
        w = p.fontMetrics().horizontalAdvance(self.status) + 22
        pill = QtCore.QRectF(r.width() - w - 10, 8, w, 20)
        c = QtGui.QColor(self.status_color)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(c.red(), c.green(), c.blue(), 60))
        p.drawRoundedRect(pill, 10, 10)
        p.setBrush(c)
        p.drawEllipse(QtCore.QPointF(pill.left() + 9, pill.center().y()), 3, 3)
        p.setPen(c)
        p.drawText(pill.adjusted(16, 0, -6, 0), QtCore.Qt.AlignmentFlag.AlignVCenter, self.status)
        p.end()


def make_plot(y_label):
    plot = pg.PlotWidget(background=CARD)
    plot.setMenuEnabled(False)
    plot.setMouseEnabled(x=False, y=False)
    plot.hideButtons()
    plot.showGrid(x=True, y=True, alpha=0.12)
    for side in ("left", "bottom"):
        axis = plot.getAxis(side)
        axis.setPen(pg.mkPen(CARD_EDGE))
        axis.setTextPen(pg.mkPen(MUTED))
    plot.setLabel("left", y_label, color=MUTED)
    plot.getAxis("left").enableAutoSIPrefix(False)
    plot.enableAutoRange(axis="y", enable=False)
    plot.setLabel("bottom", "time (s)", color=MUTED)
    plot.getPlotItem().setClipToView(True)
    plot.getPlotItem().setDownsampling(auto=True, mode="peak")
    plot.setMinimumHeight(150)
    return plot


def add_curve(plot, color, width=2.0, style=QtCore.Qt.PenStyle.SolidLine):
    return plot.plot(pen=pg.mkPen(color, width=width, style=style))


class ArmPanel(object):
    """One arm: XYZ readouts, Δ position curves and force curves."""

    def __init__(self, title):
        self.frame, body = card(title, "gripper mount · robot base frame")

        readouts = QtWidgets.QHBoxLayout()
        readouts.setSpacing(18)
        self.values = {}
        for axis, color in AXIS_COLORS.items():
            col = QtWidgets.QVBoxLayout()
            col.setSpacing(0)
            name = QtWidgets.QLabel("● {}".format(axis))
            name.setObjectName("axisName")
            name.setStyleSheet("color: {};".format(color))
            value = QtWidgets.QLabel("—")
            value.setObjectName("axisValue")
            unit = QtWidgets.QLabel("mm")
            unit.setObjectName("axisUnit")
            col.addWidget(name)
            col.addWidget(value)
            col.addWidget(unit)
            readouts.addLayout(col)
            self.values[axis] = value
        readouts.addStretch(1)
        col = QtWidgets.QVBoxLayout()
        col.setSpacing(0)
        name = QtWidgets.QLabel("● |F|")
        name.setObjectName("axisName")
        name.setStyleSheet("color: {};".format(MAG_COLOR))
        self.force_value = QtWidgets.QLabel("—")
        self.force_value.setObjectName("axisValue")
        unit = QtWidgets.QLabel("N contact force")
        unit.setObjectName("axisUnit")
        col.addWidget(name)
        col.addWidget(self.force_value)
        col.addWidget(unit)
        readouts.addLayout(col)
        body.addLayout(readouts)

        self.pos_plot = make_plot("Δ position (mm)")
        self.force_plot = make_plot("force (N)")
        self.force_plot.setXLink(self.pos_plot)
        self.pos_plot.setYRange(-MIN_SPAN["pos"], MIN_SPAN["pos"])
        self.force_plot.setYRange(-MIN_SPAN["force"], MIN_SPAN["force"])
        self.pos_curves = {a: add_curve(self.pos_plot, c) for a, c in AXIS_COLORS.items()}
        self.force_curves = {a: add_curve(self.force_plot, c) for a, c in AXIS_COLORS.items()}
        self.force_curves["|F|"] = add_curve(self.force_plot, MAG_COLOR, 1.4, QtCore.Qt.PenStyle.DashLine)
        body.addWidget(self.pos_plot, 1)
        body.addWidget(self.force_plot, 1)


class History(object):
    """Time-stamped vectors for the last `keep` seconds, thinned to at most `rate` per second,
    in preallocated arrays so drawing does not convert Python lists every frame."""

    def __init__(self, keep, width, rate=PLOT_RATE):
        self.keep, self.min_dt = keep, 1.0 / rate
        self.cap = int(keep * rate * 2) + 16
        self.t = np.empty(self.cap)
        self.v = np.empty((self.cap, width))
        self.n = 0

    def extend(self, samples):
        for s in samples:
            if self.n and s[0] - self.t[self.n - 1] < self.min_dt:
                continue
            if self.n == self.cap:                       # full: keep the newest half
                half = self.cap // 2
                self.t[:half] = self.t[self.n - half:self.n]
                self.v[:half] = self.v[self.n - half:self.n]
                self.n = half
            self.t[self.n] = s[0]
            self.v[self.n] = s[2]
            self.n += 1

    def arrays(self, since):
        """(t, v) views of the samples newer than `since`."""
        i = int(np.searchsorted(self.t[:self.n], since))
        return self.t[i:self.n], self.v[i:self.n]


def fit_y(plot, arrays, min_span):
    """Y range covering the data but never narrower than +-min_span around zero."""
    lo, hi = -min_span, min_span
    for a in arrays:
        if len(a):
            lo, hi = min(lo, float(a.min())), max(hi, float(a.max()))
    pad = 0.08 * (hi - lo)
    plot.setYRange(lo - pad, hi + pad, padding=0)


# ----------------------------------------------------------------------------- window

class Dashboard(QtWidgets.QMainWindow):

    def __init__(self, robot, arms, camera, cams, ip, window_s):
        super().__init__()
        self.robot, self.arms, self.camera, self.cams = robot, arms, camera, list(cams)
        self.kin = ArmKinematics()
        self.window_s = window_s
        self.paused = False
        self.pose_ref = None                    # Δ position reference, 14 values
        self.force_tare = np.zeros(12)
        self.pose_hist = History(max(WINDOWS) + 1, 6)      # [left xyz, right xyz], metres
        self.force_hist = History(max(WINDOWS) + 1, 12)
        self.last = {"state": 0.0, "ee_force": 0.0}
        self.cam_seq = {}

        self.setWindowTitle("Tron2 · Live Telemetry")
        self.resize(1500, 980)
        root = QtWidgets.QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        page = QtWidgets.QVBoxLayout(root)
        page.setContentsMargins(18, 14, 18, 18)
        page.setSpacing(14)

        # header
        head = QtWidgets.QHBoxLayout()
        titles = QtWidgets.QVBoxLayout()
        titles.setSpacing(0)
        title = QtWidgets.QLabel("TRON2  ·  LIVE TELEMETRY")
        title.setObjectName("title")
        sub = QtWidgets.QLabel("robot {}  ·  read only".format(ip))
        sub.setObjectName("subtitle")
        titles.addWidget(title)
        titles.addWidget(sub)
        head.addLayout(titles)
        head.addSpacing(18)
        self.chips = {k: chip(k, MUTED) for k in ("joint state", "force")}
        for c in self.chips.values():
            head.addWidget(c)
        head.addStretch(1)
        self.span_buttons = {}
        for i, s in enumerate(WINDOWS):
            b = QtWidgets.QPushButton("{} s".format(s))
            b.setCheckable(True)
            b.setToolTip("plot the last {} s  [{}]".format(s, i + 1))
            b.clicked.connect(lambda _, s=s: self.set_span(s))
            head.addWidget(b)
            self.span_buttons[s] = b
        head.addSpacing(10)
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setToolTip("freeze the plots  [space]")
        self.pause_btn.clicked.connect(self.toggle_pause)
        ref_btn = QtWidgets.QPushButton("Reset Δ")
        ref_btn.setToolTip("make the current position the zero of the Δ curves  [r]")
        ref_btn.clicked.connect(self.reset_ref)
        tare_btn = QtWidgets.QPushButton("Tare force")
        tare_btn.setToolTip("zero the force with the arms free, to show contact force only  [t]")
        tare_btn.clicked.connect(self.tare)
        for b in (self.pause_btn, ref_btn, tare_btn):
            head.addWidget(b)
        page.addLayout(head)

        # cameras
        cam_row = QtWidgets.QHBoxLayout()
        cam_row.setSpacing(14)
        self.cam_views = {}
        for c in self.cams:
            view = CameraView(c)
            self.cam_views[c] = view
            cam_row.addWidget(view, 1)
        page.addLayout(cam_row, 4)

        # arms
        arm_row = QtWidgets.QHBoxLayout()
        arm_row.setSpacing(14)
        self.panels = {"left": ArmPanel("Left arm"), "right": ArmPanel("Right arm")}
        for panel in self.panels.values():
            arm_row.addWidget(panel.frame, 1)
        page.addLayout(arm_row, 6)

        for key, fn in (("Space", self.toggle_pause), ("R", self.reset_ref), ("T", self.tare),
                        ("Q", self.close), ("Escape", self.close),
                        ("1", lambda: self.set_span(WINDOWS[0])), ("2", lambda: self.set_span(WINDOWS[1])),
                        ("3", lambda: self.set_span(WINDOWS[2]))):
            QtGui.QShortcut(QtGui.QKeySequence(key), self, activated=fn)

        self.set_span(window_s)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

    # -- controls

    def set_span(self, s):
        self.window_s = s
        for k, b in self.span_buttons.items():
            b.setChecked(k == s)
        for panel in self.panels.values():
            panel.pos_plot.setXRange(-s, 0, padding=0)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.setChecked(self.paused)
        self.pause_btn.setText("Resume" if self.paused else "Pause")

    def reset_ref(self):
        last = self.robot.streams["state"].latest()
        if last is not None:
            self.pose_ref = self.kin.positions(last[2][0]).ravel()

    def tare(self):
        last = self.arms.streams["ee_force"].since(time.monotonic() - 0.5)
        if last:
            self.force_tare = np.mean([s[2] for s in last], axis=0)

    # -- update

    def tick(self):
        now = time.monotonic()
        self.update_chips(now)
        self.update_cameras(now)
        new = self.robot.streams["state"].since(self.last["state"])
        if new:
            self.last["state"] = new[-1][0]
            if not self.paused:
                # thin to the plot rate before the kinematics, then [left xyz, right xyz]
                kept, t_prev = [], -1.0
                for smp in new:
                    if smp[0] - t_prev >= self.pose_hist.min_dt:
                        kept.append(smp)
                        t_prev = smp[0]
                self.pose_hist.extend([(smp[0], None, self.kin.positions(smp[2][0]).ravel()) for smp in kept])
        new = self.arms.streams["ee_force"].since(self.last["ee_force"])
        if new:
            self.last["ee_force"] = new[-1][0]
            if not self.paused:
                self.force_hist.extend(new)
        if self.pose_ref is None:
            self.reset_ref()
        if not self.paused:
            self.update_plots(now)

    def update_chips(self, now):
        for label, stream, unit in (("joint state", self.robot.streams["state"], "Hz"),
                                    ("force", self.arms.streams["ee_force"], "Hz")):
            last = stream.latest()
            if last is None or now - last[3] > STALE_S:
                set_chip(self.chips[label], "{}  ·  no data".format(label), BAD)
            else:
                stats = stream.stats(1.0)
                rate = stats[0] if stats else 0.0
                set_chip(self.chips[label], "{}  ·  {:.0f} {}".format(label, rate, unit), OK)

    def update_cameras(self, now):
        for c, view in self.cam_views.items():
            last = self.camera.streams[c].latest()
            if last is None:
                view.set_status("no images", BAD)
                continue
            if self.cam_seq.get(c) != last[1]:
                self.cam_seq[c] = last[1]
                view.set_jpeg(last[2])
            age = now - last[3]
            stats = self.camera.streams[c].stats(2.0)
            rate = stats[0] if stats else 0.0
            if age > STALE_S:
                view.set_status("stale  {:.1f} s".format(age), BAD)
            else:
                view.set_status("{:.0f} Hz  ·  {:.0f} ms".format(rate, age * 1000), OK if rate >= 25 else WARN)

    def update_plots(self, now):
        t, pose = self.pose_hist.arrays(now - self.window_s)
        ft, force = self.force_hist.arrays(now - self.window_s)
        for side, off_p, off_f in (("left", 0, 0), ("right", 3, 6)):
            panel = self.panels[side]
            if len(t):
                xyz = pose[:, off_p:off_p + 3] * 1000.0
                ref = self.pose_ref[off_p:off_p + 3] * 1000.0 if self.pose_ref is not None else xyz[0]
                delta = xyz - ref
                for i, axis in enumerate("XYZ"):
                    panel.pos_curves[axis].setData(t - now, delta[:, i])
                    panel.values[axis].setText("{:+7.1f}".format(xyz[-1, i]))
                fit_y(panel.pos_plot, [delta], MIN_SPAN["pos"])
            if len(ft):
                f = force[:, off_f:off_f + 3] - self.force_tare[off_f:off_f + 3]
                mag = np.linalg.norm(f, axis=1)
                for i, axis in enumerate("XYZ"):
                    panel.force_curves[axis].setData(ft - now, f[:, i])
                panel.force_curves["|F|"].setData(ft - now, mag)
                fit_y(panel.force_plot, [f, mag], MIN_SPAN["force"])
                panel.force_value.setText("{:6.1f}".format(mag[-1]))
                color = OK if mag[-1] < 5 else WARN if mag[-1] < 20 else BAD
                panel.force_value.setStyleSheet("color: {};".format(color))
        for panel in self.panels.values():
            panel.pos_plot.setXRange(-self.window_s, 0, padding=0)


# ----------------------------------------------------------------------------- main

def main():
    parser = argparse.ArgumentParser(description="Live Tron2 dashboard: cameras, end-effector position and force.")
    parser.add_argument("--ip", default="10.192.1.2")
    parser.add_argument("--window", type=int, choices=WINDOWS, default=10, help="plot span in seconds")
    parser.add_argument("--cam", action="append", choices=CAMERA_NAMES,
                        help="show only these camera(s) (default: all three)")
    args = parser.parse_args()

    pg.setConfigOptions(antialias=True, foreground=MUTED, background=CARD)
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)

    robot = RobotSource(args.ip)
    arms = ArmSource(robot.robot)
    cams = args.cam or list(CAMERA_NAMES)
    camera = SdkCameraSource(robot.robot, {c: SDK_CAMERA_TOPICS[c] for c in cams})

    window = Dashboard(robot, arms, camera, cams, args.ip, args.window)
    window.show()
    return app.exec()


if __name__ == "__main__":
    # Same SDK exit problem as the collector: its threads keep calling the callbacks while
    # the interpreter shuts down, which segfaults; leave without the teardown.
    code = 0
    try:
        code = main()
    except SystemExit as e:
        if isinstance(e.code, int) or e.code is None:
            code = e.code or 0
        else:
            print(e.code, file=sys.stderr)
            code = 1
    except BaseException:
        import traceback
        traceback.print_exc()
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
