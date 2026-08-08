"""
Interactive 2D bar-mechanism editor and kinematic simulator.

Dependencies:
    pip install matplotlib numpy scipy

Run:
    python3 mechanism_editor.py

Controls are explained in the HELP_TEXT constant near the end of this file.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backend_bases import MouseButton
from matplotlib.widgets import Button, TextBox
from scipy.optimize import least_squares

APP_VERSION = "2.2 — Visible Tool Hints"


@dataclass
class Node:
    id: int
    x: float
    y: float
    fixed: bool = False
    fixed_x: Optional[float] = None
    fixed_y: Optional[float] = None

    def __post_init__(self):
        if self.fixed_x is None:
            self.fixed_x = self.x
        if self.fixed_y is None:
            self.fixed_y = self.y


@dataclass
class Bar:
    id: int
    a: int
    b: int
    length: float


@dataclass
class Motor:
    id: int
    pivot: int
    tip: int
    angle_deg: float
    speed_deg_s: float = 30.0
    enabled: bool = True
    min_angle_deg: float = 0.0
    max_angle_deg: float = 360.0
    direction: float = 1.0


@dataclass
class LinearBearing:
    id: int
    slider: int
    guide_a: int
    guide_b: int
    enabled: bool = True


@dataclass
class Actuator:
    id: int
    a: int
    b: int
    length: float
    min_length: float
    max_length: float
    speed: float = 1.0
    enabled: bool = True
    direction: float = 1.0


class MechanismModel:
    """Data model plus generic nonlinear position solver."""

    def __init__(self):
        self.nodes: Dict[int, Node] = {}
        self.bars: Dict[int, Bar] = {}
        self.motors: Dict[int, Motor] = {}
        self.bearings: Dict[int, LinearBearing] = {}
        self.actuators: Dict[int, Actuator] = {}
        self.next_node_id = 1
        self.next_bar_id = 1
        self.next_motor_id = 1
        self.next_bearing_id = 1
        self.next_actuator_id = 1
        self.last_error = ""

    def clear(self):
        self.__init__()

    def add_node(self, x: float, y: float) -> int:
        node_id = self.next_node_id
        self.next_node_id += 1
        self.nodes[node_id] = Node(node_id, float(x), float(y))
        return node_id

    def add_or_toggle_bar(self, a: int, b: int) -> Optional[int]:
        if a == b or a not in self.nodes or b not in self.nodes:
            return None
        existing = self.bar_between(a, b)
        if existing is not None:
            self.delete_bar(existing.id)
            return None
        na, nb = self.nodes[a], self.nodes[b]
        length = math.hypot(nb.x - na.x, nb.y - na.y)
        if length < 1e-8:
            return None
        bar_id = self.next_bar_id
        self.next_bar_id += 1
        self.bars[bar_id] = Bar(bar_id, a, b, length)
        return bar_id

    def bar_between(self, a: int, b: int) -> Optional[Bar]:
        for bar in self.bars.values():
            if {bar.a, bar.b} == {a, b}:
                return bar
        return None

    def delete_bar(self, bar_id: int):
        bar = self.bars.pop(bar_id, None)
        if bar is None:
            return
        for motor_id, motor in list(self.motors.items()):
            if {motor.pivot, motor.tip} == {bar.a, bar.b}:
                del self.motors[motor_id]

    def delete_node(self, node_id: int):
        if node_id not in self.nodes:
            return
        for bar_id, bar in list(self.bars.items()):
            if node_id in (bar.a, bar.b):
                self.delete_bar(bar_id)
        for motor_id, motor in list(self.motors.items()):
            if node_id in (motor.pivot, motor.tip):
                del self.motors[motor_id]
        for bearing_id, bearing in list(self.bearings.items()):
            if node_id in (bearing.slider, bearing.guide_a, bearing.guide_b):
                del self.bearings[bearing_id]
        for actuator_id, actuator in list(self.actuators.items()):
            if node_id in (actuator.a, actuator.b):
                del self.actuators[actuator_id]
        del self.nodes[node_id]

    def toggle_fixed(self, node_id: int):
        node = self.nodes[node_id]
        node.fixed = not node.fixed
        if node.fixed:
            node.fixed_x, node.fixed_y = node.x, node.y

    def add_or_toggle_motor(self, pivot: int, tip: int, speed: float) -> Optional[int]:
        bar = self.bar_between(pivot, tip)
        if bar is None:
            self.last_error = "A motor requires an existing bar between the selected nodes."
            return None
        for motor_id, motor in list(self.motors.items()):
            if motor.pivot == pivot and motor.tip == tip:
                del self.motors[motor_id]
                return None
        p, t = self.nodes[pivot], self.nodes[tip]
        angle = math.degrees(math.atan2(t.y - p.y, t.x - p.x)) % 360.0
        motor_id = self.next_motor_id
        self.next_motor_id += 1
        self.motors[motor_id] = Motor(motor_id, pivot, tip, angle, speed)
        return motor_id

    def add_or_toggle_bearing(self, slider: int, guide_a: int,
                              guide_b: int) -> Optional[int]:
        if len({slider, guide_a, guide_b}) < 3:
            self.last_error = "Bearing needs three different nodes."
            return None
        ga, gb = self.nodes[guide_a], self.nodes[guide_b]
        if math.hypot(gb.x-ga.x, gb.y-ga.y) < 1e-8:
            self.last_error = "Bearing guide nodes cannot coincide."
            return None
        for bearing_id, bearing in list(self.bearings.items()):
            if (bearing.slider == slider and
                    {bearing.guide_a, bearing.guide_b} == {guide_a, guide_b}):
                del self.bearings[bearing_id]
                return None
        bearing_id = self.next_bearing_id
        self.next_bearing_id += 1
        self.bearings[bearing_id] = LinearBearing(
            bearing_id, slider, guide_a, guide_b)
        self.last_error = ""
        return bearing_id

    def add_or_toggle_actuator(self, a: int, b: int) -> Optional[int]:
        if a == b:
            return None
        for actuator_id, actuator in list(self.actuators.items()):
            if {actuator.a, actuator.b} == {a, b}:
                del self.actuators[actuator_id]
                return None
        # An actuator is itself the variable-length link. A rigid bar between
        # the same endpoints would make every length change impossible.
        rigid_bar = self.bar_between(a, b)
        if rigid_bar is not None:
            self.delete_bar(rigid_bar.id)
        na, nb = self.nodes[a], self.nodes[b]
        length = math.hypot(nb.x-na.x, nb.y-na.y)
        if length < 1e-8:
            self.last_error = "Actuator endpoints cannot coincide."
            return None
        actuator_id = self.next_actuator_id
        self.next_actuator_id += 1
        self.actuators[actuator_id] = Actuator(
            actuator_id, a, b, length,
            max(0.01, length*0.5), length*1.5,
            max(0.1, length*0.25))
        self.last_error = ""
        return actuator_id

    def positions(self) -> Dict[int, Tuple[float, float]]:
        return {i: (n.x, n.y) for i, n in self.nodes.items()}

    def restore_positions(self, positions: Dict[int, Tuple[float, float]]):
        for node_id, xy in positions.items():
            if node_id in self.nodes:
                self.nodes[node_id].x, self.nodes[node_id].y = xy

    def _vector_layout(self):
        ids = sorted(self.nodes)
        index = {node_id: 2 * i for i, node_id in enumerate(ids)}
        q = np.empty(2 * len(ids), dtype=float)
        for node_id in ids:
            k = index[node_id]
            q[k:k + 2] = (self.nodes[node_id].x, self.nodes[node_id].y)
        return ids, index, q

    def solve(self, drag: Optional[Tuple[int, float, float]] = None,
              continuity_weight: float = 1e-10,
              ignore_motors: bool = False,
              ignore_actuators: bool = False) -> bool:
        """Solve all constraints and commit only a rigid valid result.

        The small continuity residual selects the solution nearest to the last
        pose when the mechanism is underconstrained. It does not relax the
        acceptance test for rigid bars.
        """
        if not self.nodes:
            return True
        _, index, q_previous = self._vector_layout()
        scale = max([bar.length for bar in self.bars.values()] + [1.0])
        hard_weight = 10000.0

        def xy(q, node_id):
            k = index[node_id]
            return q[k], q[k + 1]

        def residual(q):
            values: List[float] = []
            # Rigid bar constraints, normalized to coordinate units.
            for bar in self.bars.values():
                ax, ay = xy(q, bar.a)
                bx, by = xy(q, bar.b)
                values.append(hard_weight *
                              (math.hypot(bx - ax, by - ay) - bar.length))

            # Fixed joints.
            for node in self.nodes.values():
                if node.fixed:
                    x, y = xy(q, node.id)
                    values.extend((hard_weight * (x - node.fixed_x),
                                   hard_weight * (y - node.fixed_y)))

            # Absolute-angle motors. Two equations place the tip exactly at
            # pivot + L*[cos(angle), sin(angle)].
            for motor in self.motors.values():
                if ignore_motors:
                    break
                if not motor.enabled:
                    continue
                bar = self.bar_between(motor.pivot, motor.tip)
                if bar is None:
                    continue
                px, py = xy(q, motor.pivot)
                tx, ty = xy(q, motor.tip)
                angle = math.radians(motor.angle_deg)
                values.extend((
                    hard_weight * (tx - px - bar.length * math.cos(angle)),
                    hard_weight * (ty - py - bar.length * math.sin(angle)),
                ))

            # Linear bearing: slider must remain collinear with guide A-B.
            # Cross product divided by guide length is signed distance to line.
            for bearing in self.bearings.values():
                if not bearing.enabled:
                    continue
                sx, sy = xy(q, bearing.slider)
                ax, ay = xy(q, bearing.guide_a)
                bx, by = xy(q, bearing.guide_b)
                gx, gy = bx-ax, by-ay
                gl = max(math.hypot(gx, gy), 1e-12)
                line_distance = (gx*(sy-ay)-gy*(sx-ax))/gl
                values.append(hard_weight * line_distance)

            # Actuator: rigid at its current commanded length. Disabled means
            # its length stops changing, not that the cylinder disappears.
            for actuator in self.actuators.values():
                if ignore_actuators:
                    break
                ax, ay = xy(q, actuator.a)
                bx, by = xy(q, actuator.b)
                values.append(hard_weight *
                              (math.hypot(bx-ax, by-ay)-actuator.length))

            # The mouse target is a temporary positional constraint.
            if drag is not None and drag[0] in index:
                x, y = xy(q, drag[0])
                values.extend((x - drag[1], y - drag[2]))

            # q-min ||q-q_previous||²: select the nearest valid branch.
            if continuity_weight > 0:
                values.extend(np.sqrt(continuity_weight) * (q - q_previous))
            return np.asarray(values, dtype=float)

        try:
            result = least_squares(
                residual, q_previous, method="trf", xtol=1e-11,
                ftol=1e-11, gtol=1e-11, max_nfev=500,
            )
        except Exception as exc:
            self.last_error = f"Solver exception: {exc}"
            return False

        q = result.x
        # Validate hard constraints independently. An impossible pose must not
        # be accepted merely because least_squares found a compromise.
        max_bar_error = 0.0
        for bar in self.bars.values():
            ax, ay = xy(q, bar.a)
            bx, by = xy(q, bar.b)
            max_bar_error = max(max_bar_error,
                                abs(math.hypot(bx - ax, by - ay) - bar.length))
        max_fixed_error = 0.0
        for node in self.nodes.values():
            if node.fixed:
                x, y = xy(q, node.id)
                max_fixed_error = max(max_fixed_error,
                                      math.hypot(x - node.fixed_x, y - node.fixed_y))
        max_motor_error = 0.0
        for motor in self.motors.values():
            if ignore_motors:
                break
            if not motor.enabled:
                continue
            bar = self.bar_between(motor.pivot, motor.tip)
            if bar:
                px, py = xy(q, motor.pivot)
                tx, ty = xy(q, motor.tip)
                a = math.radians(motor.angle_deg)
                max_motor_error = max(max_motor_error, math.hypot(
                    tx - px - bar.length * math.cos(a),
                    ty - py - bar.length * math.sin(a)))

        max_bearing_error = 0.0
        for bearing in self.bearings.values():
            if not bearing.enabled:
                continue
            sx, sy = xy(q, bearing.slider)
            ax, ay = xy(q, bearing.guide_a)
            bx, by = xy(q, bearing.guide_b)
            gx, gy = bx-ax, by-ay
            gl = max(math.hypot(gx, gy), 1e-12)
            max_bearing_error = max(
                max_bearing_error, abs((gx*(sy-ay)-gy*(sx-ax))/gl))

        max_actuator_error = 0.0
        for actuator in self.actuators.values():
            if ignore_actuators:
                break
            ax, ay = xy(q, actuator.a)
            bx, by = xy(q, actuator.b)
            max_actuator_error = max(
                max_actuator_error,
                abs(math.hypot(bx-ax, by-ay)-actuator.length))

        tolerance = max(1e-6, scale * 1e-6)
        if (not np.all(np.isfinite(q)) or max_bar_error > tolerance or
                max_fixed_error > tolerance or max_motor_error > tolerance or
                max_bearing_error > tolerance or
                max_actuator_error > tolerance):
            self.last_error = (
                "No rigid solution: "
                f"bar={max_bar_error:.3g}, fixed={max_fixed_error:.3g}, "
                f"motor={max_motor_error:.3g}, bearing={max_bearing_error:.3g}, "
                f"actuator={max_actuator_error:.3g}"
            )
            return False

        for node_id, k in index.items():
            self.nodes[node_id].x = float(q[k])
            self.nodes[node_id].y = float(q[k + 1])
        self.last_error = ""
        return True

    def max_bar_error(self) -> float:
        error = 0.0
        for bar in self.bars.values():
            a, b = self.nodes[bar.a], self.nodes[bar.b]
            error = max(error, abs(math.hypot(b.x-a.x, b.y-a.y)-bar.length))
        return error

    def to_dict(self):
        return {
            "nodes": [asdict(x) for x in self.nodes.values()],
            "bars": [asdict(x) for x in self.bars.values()],
            "motors": [asdict(x) for x in self.motors.values()],
            "bearings": [asdict(x) for x in self.bearings.values()],
            "actuators": [asdict(x) for x in self.actuators.values()],
            "next_ids": [self.next_node_id, self.next_bar_id,
                         self.next_motor_id, self.next_bearing_id,
                         self.next_actuator_id],
        }

    def from_dict(self, data):
        self.clear()
        self.nodes = {x["id"]: Node(**x) for x in data.get("nodes", [])}
        self.bars = {x["id"]: Bar(**x) for x in data.get("bars", [])}
        self.motors = {x["id"]: Motor(**x) for x in data.get("motors", [])}
        self.bearings = {x["id"]: LinearBearing(**x)
                         for x in data.get("bearings", [])}
        self.actuators = {x["id"]: Actuator(**x)
                          for x in data.get("actuators", [])}
        if "next_ids" in data:
            values = list(data["next_ids"])
            while len(values) < 5:
                values.append(1)
            (self.next_node_id, self.next_bar_id, self.next_motor_id,
             self.next_bearing_id, self.next_actuator_id) = values[:5]
        else:
            self.next_node_id = max(self.nodes, default=0) + 1
            self.next_bar_id = max(self.bars, default=0) + 1
            self.next_motor_id = max(self.motors, default=0) + 1
            self.next_bearing_id = max(self.bearings, default=0) + 1
            self.next_actuator_id = max(self.actuators, default=0) + 1


HELP_TEXT = """EDIT
Line: choose two endpoints.
Existing bar: choose it again to delete.
Fix: click node to fix/unfix.
Rotator: click pivot, then neighbor.
Linear Bearing: click slider, guide A, guide B.
Actuator: choose two endpoint nodes.
Move: drag a node and change geometry.
Delete: click a node or bar.

SIMULATE
Run/Pause: drive all motors.
Restart: restore the starting pose.
Drag: pull any node with the mouse.
Dragging pauses and releases motors.
Impossible rigid pose: ERROR.

Motor angle is absolute to the world.
Select a driver to edit its parameters.
Save file: mechanism.json
"""


class MechanismEditor:
    NODE_PICK_PX = 13
    BAR_PICK_PX = 9

    def __init__(self):
        self.model = MechanismModel()
        self.mode = "EDIT"
        self.tool = "LINE"
        self.pending_node: Optional[int] = None
        self.pending_nodes: List[int] = []
        self.selected_driver: Optional[Tuple[str, int]] = None
        self.param_selection_loaded = None
        self.drag_node: Optional[int] = None
        self.drag_target: Optional[Tuple[int, float, float]] = None
        self.running = False
        self.error = ""
        self.sim_snapshot = None
        self.motor_speed = 30.0
        self.save_path = Path.cwd() / "mechanism.json"

        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title(
            f"General 2D Mechanism Editor {APP_VERSION}")
        self.ax = self.fig.add_axes([0.045, 0.10, 0.69, 0.82])
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-1, 11)
        self.ax.set_ylim(-1, 8)
        self.ax.grid(True, alpha=0.22)
        self.ax.set_title("EDIT — LINE")

        self.status = self.fig.text(0.055, 0.035, "", fontsize=10)
        self.mode_banner = self.fig.text(
            0.845, 0.965, f"EDIT MODE | v{APP_VERSION}", ha="center", va="center",
            fontsize=15, weight="bold",
            bbox=dict(boxstyle="round,pad=.45", facecolor="#b8dcff"))
        self.selection_text = self.fig.text(0.755, 0.685, "Selected: none",
                                            fontsize=9, weight="bold")
        self.help_text = self.fig.text(0.755, 0.255, HELP_TEXT, va="top",
                                       family="monospace", fontsize=7.4)
        self.widgets = []
        self.edit_widgets = []
        self.sim_widgets = []
        self._build_widgets()

        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.timer = self.fig.canvas.new_timer(interval=30)
        self.timer.add_callback(self.on_timer)
        self.timer.start()

        self.load_demo()
        self.draw()

    def _button(self, rect, label, callback, color="#e8e8e8", group=None):
        axis = self.fig.add_axes(rect)
        button = Button(axis, label, color=color, hovercolor="#d5e8ff")
        button.on_clicked(callback)
        self.widgets.append(button)
        if group == "edit":
            self.edit_widgets.append(button)
        elif group == "sim":
            self.sim_widgets.append(button)
        return button

    def _build_widgets(self):
        x, w, h, gap = 0.755, 0.105, 0.040, 0.008
        self.edit_mode_button = self._button([x, .90, w, h], "EDIT", lambda e: self.set_mode("EDIT"), "#cfe8ff")
        self.sim_mode_button = self._button([x+w+gap, .90, w, h], "SIMULATE", lambda e: self.set_mode("SIMULATE"), "#d8f5d0")

        labels = ["Line", "Fix", "Rotator", "Linear Bearing", "Actuator", "Move", "Delete", "Select Driver"]
        tools = ["LINE", "FIX", "ROTATOR", "BEARING", "ACTUATOR", "MOVE", "DELETE", "SELECT"]
        for i, (label, tool) in enumerate(zip(labels, tools)):
            xx = x + (i % 2) * (w + gap)
            yy = .845 - (i // 2) * (h + gap)
            self._button([xx, yy, w, h], label,
                         lambda e, t=tool: self.set_tool(t), group="edit")

        # Shared parameter editor: motor uses angle/min angle/max angle/speed;
        # actuator uses length/min length/max length/speed.
        self.param_boxes = {}
        for label, key, yy, initial in [
                ("Value ", "value", .625, "0"),
                ("Min ", "min", .580, "0"),
                ("Max ", "max", .535, "360"),
                ("Speed ", "speed", .490, "30")]:
            box_ax = self.fig.add_axes([x, yy, 2*w+gap, h])
            box = TextBox(box_ax, label, initial=initial)
            self.param_boxes[key] = box
            self.widgets.append(box)

        self._button([x, .440, w, h], "Apply Params", self.apply_parameters)
        self._button([x+w+gap, .440, w, h], "Enable/Disable", self.toggle_selected)
        self._button([x, .390, w, h], "Run/Pause", self.toggle_run, group="sim")
        self._button([x+w+gap, .390, w, h], "Restart", self.restart, group="sim")
        self._button([x, .340, w, h], "Save JSON", self.save)
        self._button([x+w+gap, .340, w, h], "Load JSON", self.load)
        self._button([x, .290, w, h], "Demo", lambda e: self.load_demo(), group="edit")
        self._button([x+w+gap, .290, w, h], "Clear", self.clear, group="edit")

    def set_speed(self, text):
        try:
            self.motor_speed = float(text)
            self.error = ""
        except ValueError:
            self.error = "Motor speed must be a number."
        self.draw()

    def set_widget_group_state(self):
        edit_active = self.mode == "EDIT"
        for widget in self.edit_widgets:
            widget.set_active(edit_active)
            widget.ax.set_facecolor("#e8e8e8" if edit_active else "#c8c8c8")
        for widget in self.sim_widgets:
            widget.set_active(not edit_active)
            widget.ax.set_facecolor("#e8e8e8" if not edit_active else "#c8c8c8")
        self.edit_mode_button.ax.set_facecolor("#70b7f0" if edit_active else "#d8d8d8")
        self.sim_mode_button.ax.set_facecolor("#77cf70" if not edit_active else "#d8d8d8")
        self.mode_banner.set_text(
            ("EDIT MODE" if edit_active else "SIMULATE MODE") +
            f" | v{APP_VERSION}")
        self.mode_banner.get_bbox_patch().set_facecolor(
            "#b8dcff" if edit_active else "#bcebb4")

    def refresh_parameter_boxes(self, force=False):
        if self.selected_driver is None:
            self.selection_text.set_text("Selected driver: none")
            self.param_selection_loaded = None
            return
        kind, driver_id = self.selected_driver
        if kind == "motor" and driver_id in self.model.motors:
            d = self.model.motors[driver_id]
            values = (d.angle_deg, d.min_angle_deg, d.max_angle_deg, d.speed_deg_s)
            self.selection_text.set_text(
                f"Selected: Motor M{driver_id} ({'ON' if d.enabled else 'OFF'})")
        elif kind == "actuator" and driver_id in self.model.actuators:
            d = self.model.actuators[driver_id]
            values = (d.length, d.min_length, d.max_length, d.speed)
            self.selection_text.set_text(
                f"Selected: Actuator A{driver_id} ({'ON' if d.enabled else 'OFF'})")
        else:
            self.selected_driver = None
            self.selection_text.set_text("Selected driver: none")
            return
        if not force and self.param_selection_loaded == self.selected_driver:
            return
        for key, value in zip(("value", "min", "max", "speed"), values):
            self.param_boxes[key].set_val(f"{value:.6g}")
        self.param_selection_loaded = self.selected_driver

    def apply_parameters(self, _event=None):
        if self.selected_driver is None:
            self.error = "Select or create a motor/actuator first."
            self.draw()
            return
        try:
            value = float(self.param_boxes["value"].text)
            minimum = float(self.param_boxes["min"].text)
            maximum = float(self.param_boxes["max"].text)
            speed = abs(float(self.param_boxes["speed"].text))
            if maximum <= minimum or speed < 1e-12:
                raise ValueError("Max must be greater than Min and Speed must be positive")
            kind, driver_id = self.selected_driver
            if kind == "motor":
                d = self.model.motors[driver_id]
                d.min_angle_deg, d.max_angle_deg = minimum, maximum
                d.angle_deg = min(max(value, minimum), maximum)
                d.speed_deg_s = speed
            else:
                d = self.model.actuators[driver_id]
                if minimum <= 0:
                    raise ValueError("Actuator Min must be positive")
                d.min_length, d.max_length = minimum, maximum
                d.length = min(max(value, minimum), maximum)
                d.speed = speed
            self.error = "Parameters applied."
            self.refresh_parameter_boxes(force=True)
        except (ValueError, KeyError) as exc:
            self.error = f"Invalid parameters: {exc}"
        self.draw()

    def toggle_selected(self, _event=None):
        if self.selected_driver is None:
            self.error = "Select or create a motor/actuator first."
        else:
            kind, driver_id = self.selected_driver
            collection = self.model.motors if kind == "motor" else self.model.actuators
            if driver_id in collection:
                collection[driver_id].enabled = not collection[driver_id].enabled
                self.error = "Driver state changed."
            else:
                self.selected_driver = None
                self.error = "Selected driver no longer exists."
        self.draw()

    def set_mode(self, mode):
        self.mode = mode
        self.running = False
        self.pending_node = None
        self.pending_nodes = []
        self.drag_node = None
        self.drag_target = None
        self.error = ""
        if mode == "SIMULATE":
            self.sim_snapshot = (
                self.model.positions(),
                {i: m.angle_deg for i, m in self.model.motors.items()},
                {i: a.length for i, a in self.model.actuators.items()},
            )
        self.draw()

    def set_tool(self, tool):
        if self.mode != "EDIT":
            self.error = "Editing tools are available only in EDIT mode."
        else:
            self.tool = tool
            self.pending_node = None
            self.pending_nodes = []
            self.error = ""
        self.draw()

    def toggle_run(self, _event):
        if self.mode != "SIMULATE":
            self.set_mode("SIMULATE")
        self.running = not self.running
        self.draw()

    def restart(self, _event=None):
        self.running = False
        if self.sim_snapshot:
            positions, angles, lengths = self.sim_snapshot
            self.model.restore_positions(positions)
            for motor_id, angle in angles.items():
                if motor_id in self.model.motors:
                    self.model.motors[motor_id].angle_deg = angle
            for actuator_id, length in lengths.items():
                if actuator_id in self.model.actuators:
                    self.model.actuators[actuator_id].length = length
        self.error = ""
        self.draw()

    def save(self, _event=None):
        try:
            self.save_path.write_text(json.dumps(self.model.to_dict(), indent=2), encoding="utf-8")
            self.error = f"Saved: {self.save_path.name}"
        except Exception as exc:
            self.error = f"Save failed: {exc}"
        self.draw()

    def load(self, _event=None):
        try:
            data = json.loads(self.save_path.read_text(encoding="utf-8"))
            self.model.from_dict(data)
            self.selected_driver = None
            self.pending_node = None
            self.running = False
            self.error = f"Loaded: {self.save_path.name}"
        except Exception as exc:
            self.error = f"Load failed: {exc}"
        self.draw()

    def clear(self, _event=None):
        self.model.clear()
        self.selected_driver = None
        self.pending_node = None
        self.running = False
        self.error = ""
        self.draw()

    def load_demo(self):
        self.model.clear()
        # A scaled five-bar example with a useful reachable motor range.
        coords = [(1.0, 1.0), (1.975, 2.392), (3.8, 3.95),
                  (5.625, 2.392), (6.6, 1.0)]
        ids = [self.model.add_node(x, y) for x, y in coords]
        for a, b in zip(ids, ids[1:]):
            self.model.add_or_toggle_bar(a, b)
        self.model.add_or_toggle_bar(ids[-1], ids[0])
        self.model.toggle_fixed(ids[0])
        self.model.toggle_fixed(ids[-1])
        self.model.add_or_toggle_motor(ids[0], ids[1], self.motor_speed)
        self.selected_driver = ("motor", 1)
        self.pending_node = None
        self.running = False
        self.mode = "EDIT"
        self.error = "Demo loaded."
        self.draw()

    def data_distance_px(self, x1, y1, x2, y2):
        p1 = self.ax.transData.transform((x1, y1))
        p2 = self.ax.transData.transform((x2, y2))
        return float(np.hypot(*(p2-p1)))

    def pick_node(self, event) -> Optional[int]:
        if event.inaxes != self.ax or event.xdata is None:
            return None
        best = None
        for node in self.model.nodes.values():
            d = self.data_distance_px(event.xdata, event.ydata, node.x, node.y)
            if d <= self.NODE_PICK_PX and (best is None or d < best[0]):
                best = (d, node.id)
        return None if best is None else best[1]

    def pick_bar(self, event) -> Optional[int]:
        if event.inaxes != self.ax or event.xdata is None:
            return None
        mouse = np.array([event.x, event.y], dtype=float)
        best = None
        for bar in self.model.bars.values():
            a, b = self.model.nodes[bar.a], self.model.nodes[bar.b]
            p = self.ax.transData.transform((a.x, a.y))
            q = self.ax.transData.transform((b.x, b.y))
            v = q-p
            t = 0.0 if np.dot(v, v) == 0 else np.clip(np.dot(mouse-p, v)/np.dot(v, v), 0, 1)
            d = float(np.linalg.norm(mouse-(p+t*v)))
            if d <= self.BAR_PICK_PX and (best is None or d < best[0]):
                best = (d, bar.id)
        return None if best is None else best[1]

    def pick_actuator(self, event) -> Optional[int]:
        if event.inaxes != self.ax or event.xdata is None:
            return None
        mouse = np.array([event.x, event.y], dtype=float)
        best = None
        for actuator in self.model.actuators.values():
            a, b = self.model.nodes[actuator.a], self.model.nodes[actuator.b]
            p = self.ax.transData.transform((a.x, a.y))
            q = self.ax.transData.transform((b.x, b.y))
            v = q-p
            t = 0.0 if np.dot(v, v) == 0 else np.clip(np.dot(mouse-p, v)/np.dot(v, v), 0, 1)
            d = float(np.linalg.norm(mouse-(p+t*v)))
            if d <= self.BAR_PICK_PX+4 and (best is None or d < best[0]):
                best = (d, actuator.id)
        return None if best is None else best[1]

    def get_or_create_node(self, event) -> Optional[int]:
        node_id = self.pick_node(event)
        if node_id is not None:
            return node_id
        if event.inaxes == self.ax and event.xdata is not None:
            return self.model.add_node(event.xdata, event.ydata)
        return None

    def on_press(self, event):
        if event.button != MouseButton.LEFT or event.inaxes != self.ax:
            return
        node_id = self.pick_node(event)

        if self.mode == "SIMULATE":
            if node_id is not None:
                if self.model.nodes[node_id].fixed:
                    self.error = "This node is fixed. Unfix it in EDIT mode before dragging."
                    self.draw()
                    return
                # Manual manipulation owns the free coordinates. A running
                # motor would otherwise lock its driven tip and overconstrain
                # most dragged nodes.
                self.running = False
                self.drag_node = node_id
                self.drag_target = (node_id, event.xdata, event.ydata)
            else:
                actuator_id = self.pick_actuator(event)
                if actuator_id is not None:
                    self.selected_driver = ("actuator", actuator_id)
                    self.error = "Actuator selected."
                else:
                    bar_id = self.pick_bar(event)
                    if bar_id is not None:
                        bar = self.model.bars[bar_id]
                        for motor in self.model.motors.values():
                            if {motor.pivot, motor.tip} == {bar.a, bar.b}:
                                self.selected_driver = ("motor", motor.id)
                                self.error = "Motor selected."
                                break
                self.draw()
            return

        if self.tool == "LINE":
            clicked = self.get_or_create_node(event)
            if clicked is None:
                return
            if self.pending_node is None:
                self.pending_node = clicked
            elif clicked == self.pending_node:
                self.pending_node = None
            else:
                self.model.add_or_toggle_bar(self.pending_node, clicked)
                self.pending_node = clicked  # continue a polyline
            self.error = ""

        elif self.tool == "FIX" and node_id is not None:
            self.model.toggle_fixed(node_id)
            self.error = ""

        elif self.tool == "ROTATOR" and node_id is not None:
            if self.pending_node is None:
                self.pending_node = node_id
            elif node_id == self.pending_node:
                self.pending_node = None
            else:
                motor_id = self.model.add_or_toggle_motor(
                    self.pending_node, node_id, self.motor_speed)
                if motor_id is not None:
                    self.selected_driver = ("motor", motor_id)
                self.error = self.model.last_error
                self.pending_node = None

        elif self.tool == "BEARING" and node_id is not None:
            self.pending_nodes.append(node_id)
            self.pending_node = node_id
            if len(self.pending_nodes) == 3:
                self.model.add_or_toggle_bearing(*self.pending_nodes)
                self.error = self.model.last_error
                self.pending_nodes = []
                self.pending_node = None

        elif self.tool == "ACTUATOR" and node_id is not None:
            if self.pending_node is None:
                self.pending_node = node_id
            elif node_id == self.pending_node:
                self.pending_node = None
            else:
                actuator_id = self.model.add_or_toggle_actuator(
                    self.pending_node, node_id)
                if actuator_id is not None:
                    self.selected_driver = ("actuator", actuator_id)
                self.error = self.model.last_error
                self.pending_node = None

        elif self.tool == "SELECT":
            actuator_id = self.pick_actuator(event)
            if actuator_id is not None:
                self.selected_driver = ("actuator", actuator_id)
                self.error = "Actuator selected."
            else:
                bar_id = self.pick_bar(event)
                selected = None
                if bar_id is not None:
                    bar = self.model.bars[bar_id]
                    for motor in self.model.motors.values():
                        if {motor.pivot, motor.tip} == {bar.a, bar.b}:
                            selected = ("motor", motor.id)
                            break
                self.selected_driver = selected
                self.error = "Motor selected." if selected else "No driver at click."

        elif self.tool == "MOVE" and node_id is not None:
            self.drag_node = node_id

        elif self.tool == "DELETE":
            if node_id is not None:
                self.model.delete_node(node_id)
                if self.pending_node == node_id:
                    self.pending_node = None
            else:
                actuator_id = self.pick_actuator(event)
                if actuator_id is not None:
                    del self.model.actuators[actuator_id]
                    if self.selected_driver == ("actuator", actuator_id):
                        self.selected_driver = None
                else:
                    bar_id = self.pick_bar(event)
                    if bar_id is not None:
                        self.model.delete_bar(bar_id)
        self.draw()

    def on_motion(self, event):
        if self.drag_node is None or event.inaxes != self.ax or event.xdata is None:
            return
        if self.mode == "EDIT" and self.tool == "MOVE":
            node = self.model.nodes.get(self.drag_node)
            if node and not node.fixed:
                node.x, node.y = event.xdata, event.ydata
                # Editing changes geometry: connected bar lengths follow the edit.
                for bar in self.model.bars.values():
                    if self.drag_node in (bar.a, bar.b):
                        a, b = self.model.nodes[bar.a], self.model.nodes[bar.b]
                        bar.length = math.hypot(b.x-a.x, b.y-a.y)
                self.error = ""
        elif self.mode == "SIMULATE":
            before = self.model.positions()
            self.drag_target = (self.drag_node, event.xdata, event.ydata)
            if not self.model.solve(self.drag_target, ignore_motors=True,
                                    ignore_actuators=True):
                self.model.restore_positions(before)
                self.error = self.model.last_error
            else:
                self.error = ""
        self.draw()

    def on_release(self, event):
        if self.drag_node is not None and self.mode == "SIMULATE":
            # Keep the solved pose, then remove the temporary mouse constraint.
            self.drag_target = None
            # Continue every motor from the manually selected configuration,
            # avoiding a jump back to its old commanded angle on the next Run.
            for motor in self.model.motors.values():
                pivot = self.model.nodes.get(motor.pivot)
                tip = self.model.nodes.get(motor.tip)
                if pivot and tip:
                    motor.angle_deg = math.degrees(
                        math.atan2(tip.y-pivot.y, tip.x-pivot.x)
                    ) % 360.0
            for actuator in self.model.actuators.values():
                a = self.model.nodes.get(actuator.a)
                b = self.model.nodes.get(actuator.b)
                if a and b:
                    actuator.length = math.hypot(b.x-a.x, b.y-a.y)
                    # Preserve the manually chosen rigid pose. If the user
                    # dragged beyond the old stroke, extend the editable range.
                    actuator.min_length = min(actuator.min_length, actuator.length)
                    actuator.max_length = max(actuator.max_length, actuator.length)
        self.drag_node = None

    def on_timer(self):
        if not self.running or self.mode != "SIMULATE":
            return
        before_positions = self.model.positions()
        dt = 0.030
        for motor in self.model.motors.values():
            if motor.enabled:
                span = motor.max_angle_deg-motor.min_angle_deg
                motor.angle_deg += motor.direction*motor.speed_deg_s*dt
                if span >= 359.999:
                    motor.angle_deg = ((motor.angle_deg-motor.min_angle_deg) % span
                                       + motor.min_angle_deg)
                elif motor.angle_deg >= motor.max_angle_deg:
                    motor.angle_deg = motor.max_angle_deg
                    motor.direction = -1.0
                elif motor.angle_deg <= motor.min_angle_deg:
                    motor.angle_deg = motor.min_angle_deg
                    motor.direction = 1.0
        for actuator in self.model.actuators.values():
            if actuator.enabled:
                actuator.length += actuator.direction*actuator.speed*dt
                if actuator.length >= actuator.max_length:
                    actuator.length = actuator.max_length
                    actuator.direction = -1.0
                elif actuator.length <= actuator.min_length:
                    actuator.length = actuator.min_length
                    actuator.direction = 1.0
        if not self.model.solve(self.drag_target):
            self.model.restore_positions(before_positions)
            # Keep commanded motor angles moving so a later reachable angle can recover.
            self.error = self.model.last_error
        else:
            self.error = ""
        self.draw()

    def current_tool_hint(self):
        if self.mode == "SIMULATE":
            return ("Drag any non-fixed node. Run/Pause drives motors and actuators; "
                    "click a driver line to select its parameters.")
        if self.tool == "LINE":
            return ("LINE — click the first endpoint, then the second endpoint. "
                    "Click an existing bar pair again to remove it.")
        if self.tool == "FIX":
            return "FIX — click a node to fix or unfix it. Fixed nodes are black squares."
        if self.tool == "ROTATOR":
            return ("ROTATOR — STEP 1: select the pivot node."
                    if self.pending_node is None else
                    "ROTATOR — STEP 2: select a neighboring node connected by a bar.")
        if self.tool == "BEARING":
            step = len(self.pending_nodes)
            return [
                "LINEAR BEARING — STEP 1: select the node that must slide.",
                "LINEAR BEARING — STEP 2: select the first guide-line node.",
                "LINEAR BEARING — STEP 3: select the second guide-line node.",
            ][min(step, 2)]
        if self.tool == "ACTUATOR":
            return ("ACTUATOR — STEP 1: select the first endpoint."
                    if self.pending_node is None else
                    "ACTUATOR — STEP 2: select the second endpoint.")
        if self.tool == "MOVE":
            return "MOVE — drag a non-fixed node; connected edit lengths update."
        if self.tool == "DELETE":
            return "DELETE — click a node, ordinary bar, or actuator to remove it."
        if self.tool == "SELECT":
            return "SELECT DRIVER — click a red motor bar or a green actuator."
        return ""

    def draw(self):
        ax = self.ax
        old_xlim, old_ylim = ax.get_xlim(), ax.get_ylim()
        ax.clear()
        ax.set_xlim(old_xlim)
        ax.set_ylim(old_ylim)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.22)
        ax.set_title(
            f"{self.mode} — {self.tool}" + (" — RUNNING" if self.running else ""),
            fontsize=12, pad=12)
        ax.text(
            0.015, 0.985, self.current_tool_hint(),
            transform=ax.transAxes, ha="left", va="top", zorder=20,
            fontsize=10.5, weight="bold", color="#3b2f00",
            bbox=dict(boxstyle="round,pad=.55", facecolor="#fff2a8",
                      edgecolor="#d19a00", linewidth=2, alpha=.97))

        motor_pairs = {(m.pivot, m.tip) for m in self.model.motors.values()}
        for bar in self.model.bars.values():
            a, b = self.model.nodes[bar.a], self.model.nodes[bar.b]
            is_motor = (bar.a, bar.b) in motor_pairs or (bar.b, bar.a) in motor_pairs
            ax.plot([a.x, b.x], [a.y, b.y], color="#e45756" if is_motor else "#3973ac",
                    lw=6, solid_capstyle="round", zorder=2)
            mx, my = (a.x+b.x)/2, (a.y+b.y)/2
            ax.text(mx, my, f"L={bar.length:.2f}", fontsize=8, color="#444")

        for bearing in self.model.bearings.values():
            slider = self.model.nodes[bearing.slider]
            ga = self.model.nodes[bearing.guide_a]
            gb = self.model.nodes[bearing.guide_b]
            color = "#8e44ad" if bearing.enabled else "#aaa"
            ax.plot([ga.x, gb.x], [ga.y, gb.y], ls="--", lw=3,
                    color=color, zorder=1)
            ax.scatter(slider.x, slider.y, s=260, marker="D",
                       facecolor="none", edgecolor=color, linewidth=2.5, zorder=3)
            ax.text(slider.x, slider.y-0.38, f"Bearing B{bearing.id}",
                    color=color, ha="center", fontsize=8)

        for actuator in self.model.actuators.values():
            a, b = self.model.nodes[actuator.a], self.model.nodes[actuator.b]
            color = "#159447" if actuator.enabled else "#999"
            ax.plot([a.x, b.x], [a.y, b.y], color=color, lw=9,
                    alpha=.75, solid_capstyle="butt", zorder=2.5)
            ax.plot([a.x, b.x], [a.y, b.y], color="white", lw=2,
                    alpha=.8, zorder=2.6)
            mx, my = (a.x+b.x)/2, (a.y+b.y)/2
            ax.text(mx, my-0.22,
                    f"A{actuator.id}: L={actuator.length:.2f} "
                    f"[{actuator.min_length:.2f},{actuator.max_length:.2f}]",
                    color=color, ha="center", fontsize=8, weight="bold")

        for node in self.model.nodes.values():
            if node.fixed:
                ax.scatter(node.x, node.y, s=180, marker="s", c="#222", zorder=4)
            else:
                ax.scatter(node.x, node.y, s=145, marker="o", facecolor="white",
                           edgecolor="#111", linewidth=2, zorder=4)
            color = "#ffb000" if node.id == self.pending_node else "#111"
            ax.text(node.x, node.y+0.22, f"N{node.id}", ha="center", color=color,
                    weight="bold", zorder=5)

        for motor in self.model.motors.values():
            p = self.model.nodes[motor.pivot]
            ax.text(p.x, p.y-0.30,
                    f"M{motor.id}: θ={motor.angle_deg:.1f}°  ω={motor.speed_deg_s:g}°/s",
                    ha="center", color="#c62828", fontsize=8, weight="bold")

        self.set_widget_group_state()
        self.refresh_parameter_boxes()

        if self.error:
            self.status.set_text("ERROR / INFO: " + self.error)
            self.status.set_color("#c62828" if "failed" in self.error.lower() or
                                  "no rigid" in self.error.lower() or
                                  "requires" in self.error.lower() else "#333")
        else:
            self.status.set_text(
                f"Nodes {len(self.model.nodes)} | Bars {len(self.model.bars)} | "
                f"Motors {len(self.model.motors)} | Actuators {len(self.model.actuators)} | "
                f"Bearings {len(self.model.bearings)} | Max bar error "
                f"{self.model.max_bar_error():.3e}"
            )
            self.status.set_color("#187a2f")
        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


if __name__ == "__main__":
    MechanismEditor().show()
