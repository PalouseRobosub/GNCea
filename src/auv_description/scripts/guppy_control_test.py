#!/usr/bin/env python3
import sys
import threading
from dataclasses import dataclass, field
from typing import Dict
from geometry_msgs.msg import Vector3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# PyQt5 GUI
from PyQt5 import QtCore, QtWidgets

# ----------------------- Config -----------------------

THRUSTERS = ["t0C", "t0M", "t1C", "t1M", "t2C", "t2M", "t3C", "t3M"]
TOPIC_PREFIX = "/guppy/thrusters"  # final topics: /guppy/thrusters/t0C/force, /torque

# ----------------------- ROS Node ---------------------

@dataclass
class WrenchCmd:
    force: Vector3 = field(default_factory=Vector3)
    torque: Vector3 = field(default_factory=Vector3)

class ThrusterTeleopNode(Node):
    def __init__(self):
        super().__init__("slider_auv_teleop")

        # Parameters
        self.declare_parameter("topic_prefix", TOPIC_PREFIX)
        self.declare_parameter("thrusters", THRUSTERS)
        self.declare_parameter("force_limit", 100.0)   # slider max abs [N]
        self.declare_parameter("torque_limit", 50.0)   # slider max abs [N m]
        self.declare_parameter("rate_hz", 20.0)        # republish rate

        self.topic_prefix: str = self.get_parameter("topic_prefix").get_parameter_value().string_value
        self.thrusters: list = list(self.get_parameter("thrusters").value or THRUSTERS)
        self.force_limit: float = float(self.get_parameter("force_limit").value)
        self.torque_limit: float = float(self.get_parameter("torque_limit").value)
        self.rate_hz: float = max(1.0, float(self.get_parameter("rate_hz").value))

        # Publishers per thruster
        self.force_pubs: Dict[str, any] = {}
        self.torque_pubs: Dict[str, any] = {}
        for t in self.thrusters:
            f_topic = f"{self.topic_prefix}/{t}/force"
            tau_topic = f"{self.topic_prefix}/{t}/torque"
            self.force_pubs[t] = self.create_publisher(Vector3, f_topic, 10)
            self.torque_pubs[t] = self.create_publisher(Vector3, tau_topic, 10)
            self.get_logger().info(f"Publishing to {f_topic} and {tau_topic}")

        # Current commands per thruster
        self.cmds: Dict[str, WrenchCmd] = {t: WrenchCmd(Vector3(), Vector3()) for t in self.thrusters}

        # Periodic publish
        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

    def set_force(self, thruster: str, x: float, y: float, z: float):
        c = self.cmds[thruster]
        c.force.x, c.force.y, c.force.z = float(x), float(y), float(z)

    def set_torque(self, thruster: str, x: float, y: float, z: float):
        c = self.cmds[thruster]
        c.torque.x, c.torque.y, c.torque.z = float(x), float(y), float(z)

    def zero_all(self):
        for t in self.thrusters:
            self.set_force(t, 0.0, 0.0, 0.0)
            self.set_torque(t, 0.0, 0.0, 0.0)

    def _on_timer(self):
        for t in self.thrusters:
            self.force_pubs[t].publish(self.cmds[t].force)
            self.torque_pubs[t].publish(self.cmds[t].torque)

# ----------------------- Qt Widgets -------------------

class VectorSlider(QtWidgets.QWidget):
    """Three sliders for X/Y/Z with labels; values in physical units."""
    changed = QtCore.pyqtSignal(float, float, float)

    def __init__(self, title: str, max_abs: float, parent=None):
        super().__init__(parent)
        self.max_abs = max_abs

        box = QtWidgets.QGroupBox(title)
        grid = QtWidgets.QGridLayout(box)

        self._sx = self._make_slider()
        self._sy = self._make_slider()
        self._sz = self._make_slider()
        self._lx = QtWidgets.QLabel("0.0")
        self._ly = QtWidgets.QLabel("0.0")
        self._lz = QtWidgets.QLabel("0.0")

        grid.addWidget(QtWidgets.QLabel("X"), 0, 0)
        grid.addWidget(self._sx, 0, 1)
        grid.addWidget(self._lx, 0, 2)
        grid.addWidget(QtWidgets.QLabel("Y"), 1, 0)
        grid.addWidget(self._sy, 1, 1)
        grid.addWidget(self._ly, 1, 2)
        grid.addWidget(QtWidgets.QLabel("Z"), 2, 0)
        grid.addWidget(self._sz, 2, 1)
        grid.addWidget(self._lz, 2, 2)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(box)

        self._sx.valueChanged.connect(self._emit)
        self._sy.valueChanged.connect(self._emit)
        self._sz.valueChanged.connect(self._emit)

        self.set_all(0, 0, 0)

    def _make_slider(self) -> QtWidgets.QSlider:
        s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        s.setRange(-1000, 1000)  # map to [-max_abs, +max_abs]
        s.setSingleStep(5)
        s.setPageStep(50)
        s.setValue(0)
        return s

    def _emit(self):
        x = self._sx.value() / 1000.0 * self.max_abs
        y = self._sy.value() / 1000.0 * self.max_abs
        z = self._sz.value() / 1000.0 * self.max_abs
        self._lx.setText(f"{x:.1f}")
        self._ly.setText(f"{y:.1f}")
        self._lz.setText(f"{z:.1f}")
        self.changed.emit(x, y, z)

    def set_all(self, x: float, y: float, z: float):
        def to_ticks(v): return int(max(-self.max_abs, min(self.max_abs, v)) / self.max_abs * 1000.0)
        self._sx.blockSignals(True); self._sy.blockSignals(True); self._sz.blockSignals(True)
        self._sx.setValue(to_ticks(x)); self._sy.setValue(to_ticks(y)); self._sz.setValue(to_ticks(z))
        self._sx.blockSignals(False); self._sy.blockSignals(False); self._sz.blockSignals(False)
        self._emit()

class ThrusterRow(QtWidgets.QWidget):
    """One thruster: Force sliders + optional torque sliders."""
    def __init__(self, thruster: str, node: ThrusterTeleopNode, parent=None):
        super().__init__(parent)
        self.t = thruster
        self.node = node

        layout = QtWidgets.QHBoxLayout(self)
        self.force = VectorSlider(f"{thruster}  Force [N]", node.force_limit)
        self.torque = VectorSlider(f"{thruster}  Torque [N·m]", node.torque_limit)
        self.torque.setVisible(False)  # hidden by default; toggleable

        self.force.changed.connect(lambda x, y, z: node.set_force(self.t, x, y, z))
        self.torque.changed.connect(lambda x, y, z: node.set_torque(self.t, x, y, z))

        layout.addWidget(self.force)
        layout.addWidget(self.torque)

    def show_torque(self, visible: bool):
        self.torque.setVisible(visible)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: ThrusterTeleopNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("AUV Thruster Teleop (sliders)")

        central = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(central)

        # Controls
        controls = QtWidgets.QHBoxLayout()
        self.chk_torque = QtWidgets.QCheckBox("Show torque sliders")
        self.btn_zero = QtWidgets.QPushButton("Zero ALL")
        self.btn_zero.setShortcut("Space")
        controls.addWidget(self.chk_torque)
        controls.addStretch(1)
        controls.addWidget(self.btn_zero)
        v.addLayout(controls)

        # Thruster rows in a grid: 4 x 2
        grid = QtWidgets.QGridLayout()
        self.rows: Dict[str, ThrusterRow] = {}
        for i, t in enumerate(node.thrusters):
            row = ThrusterRow(t, node)
            self.rows[t] = row
            grid.addWidget(row, i // 2, i % 2)

        v.addLayout(grid)
        self.setCentralWidget(central)

        # Signals
        self.chk_torque.toggled.connect(self._on_toggle_torque)
        self.btn_zero.clicked.connect(self._on_zero_all)

        # Status
        self.statusBar().showMessage(f"Publishing at {node.rate_hz:.0f} Hz")

    def _on_toggle_torque(self, on: bool):
        for r in self.rows.values():
            r.show_torque(on)

    def _on_zero_all(self):
        self.node.zero_all()
        for r in self.rows.values():
            r.force.set_all(0, 0, 0)
            r.torque.set_all(0, 0, 0)

# ----------------------- Spin Qt + rclpy -------------------

def spin_ros(node: Node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

def main():
    rclpy.init()
    node = ThrusterTeleopNode()

    # Start ROS in a background thread so Qt event loop stays responsive
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(node)
    win.resize(1100, 600)
    win.show()

    code = app.exec_()
    node.get_logger().info("GUI closed; shutting down.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)

if __name__ == "__main__":
    main()
